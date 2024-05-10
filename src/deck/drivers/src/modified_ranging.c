#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "debug.h"
#include "log.h"

#include "adhocdeck.h"
#include "timers.h"
#include "static_mem.h"

#include "modified_ranging.h"
#include "ranging_table.h"

#ifndef UWB_MODIFIED_RANGING_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

static QueueHandle_t rxQueue;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static uint16_t MY_UWB_ADDRESS; // 本机地址
static RangingMODE mode = CLASSIC_MODE; // 测距模式

static RangingTableSet_t rangingTableSet; // 测距表集合
static int rangingSeqNumber = 1; // 本地测距处理序号
static int loopIndex = 0; // 遍历邻居测距表下标

static dwTime_t nullTimeStamp = {.full = NULL_TIMESTAMP};

/*
typedef struct
{
    int size;                                      // 邻居数量
    SemaphoreHandle_t mu;                          // 互斥量
    RangingTable_t TaRbTable;                    // 本地发送表
    RangingTable_t TbRaTables[MAX_NEIGHBOR_NUM]; // 邻居响应表
}RangingTableSet_t;
*/

//初始化测距表
void initRangingTableSet(){
    rangingTableSet.size = 0;
    rangingTableSet.mu = xSemaphoreCreateMutex();
    rangingTableSet.sendBufferTop = -1;
    for (table_index_t i = 0; i < TABLE_BUFFER_SIZE; i++)
    {
        rangingTableSet.sendBuffer[i].timestamp.full = NULL_TIMESTAMP;
        rangingTableSet.sendBuffer[i].seqNumber = NULL_SEQ;
    }
    for (int i = 0; i < MAX_NEIGHBOR_NUM; i++)
    {
        initRangingTable(&rangingTableSet.neighbor[i]);
    }
}

// 注册测距表
table_index_t registerRangingTable(uint16_t address){
    table_index_t index = findRangingTable(address);
    if(index != NULL_INDEX){
        DEBUG_PRINT("registerRangingTable: Ranging table already exists.\n");
        return index;
    }
    // 插入最后
    if(rangingTableSet.size < MAX_NEIGHBOR_NUM){
        enableRangingTable(&rangingTableSet.neighbor[rangingTableSet.size], address);
        index = rangingTableSet.size;
        rangingTableSet.size++;
    }
    else{
        DEBUG_PRINT("registerRangingTable: Ranging table set is full, cannot register new table.\n");
        return NULL_INDEX;
    }
    return index;
}

// 注销测距表
void unregisterRangingTable(uint16_t address){
    table_index_t index = findRangingTable(address);
    if(index == NULL_INDEX){
        DEBUG_PRINT("unregisterRangingTable: Ranging table does not exist.\n");
        return;
    }
    // 测距表前移, 释放最后一个
    for(int i = index; i < rangingTableSet.size - 1; i++){
        memcpy(&rangingTableSet.neighbor[i], &rangingTableSet.neighbor[i + 1], sizeof(RangingTable_t));
    }
    disableRangingTable(&rangingTableSet.neighbor[rangingTableSet.size - 1]);
    rangingTableSet.size--;
}

// 查找指定邻居测距表下标
table_index_t findRangingTable(uint16_t address){
    for (table_index_t i = 0; i < rangingTableSet.size; i++)
    {
        if(rangingTableSet.neighbor[i].address == address){
            return i;
        }
    }
    return NULL_INDEX;
}

// 查找指定序号发送缓冲区下标
table_index_t findSendBufferIndex(uint16_t seq){
    table_index_t index = rangingTableSet.sendBufferTop;
    int count = 0;
    while (rangingTableSet.sendBuffer[index].seqNumber != NULL_SEQ && count < rangingTableSet.size)
    {
        if(rangingTableSet.sendBuffer[index].seqNumber == seq){
            return index;
        }
        index = (index - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
    }
    return NULL_INDEX;

}
// 打印测距消息
static void printfRangingMessage(Ranging_Message_t* rangingMessage){
    // 打印报文全内容
    DEBUG_PRINT("Ranging Message:\n");
    DEBUG_PRINT("Header:\n");
    /*
    typedef struct
    {
        uint16_t srcAddress;                                     // 2 byte
        uint16_t msgSequence;                                    // 2 byte
        Timestamp_Tuple_t lastTxTimestamps[RANGING_MAX_Tr_UNIT]; // 10 byte * MAX_Tr_UNIT
        uint16_t msgLength;                                      // 2 byte
        //   uint16_t filter; // 16 bits bloom filter //暂时不知作用
    } __attribute__((packed)) Ranging_Message_Header_t; // 36
    */
    DEBUG_PRINT("srcAddress: %u\n", rangingMessage->header.srcAddress);
    DEBUG_PRINT("msgSequence: %u\n", rangingMessage->header.msgSequence);
    DEBUG_PRINT("msgLength: %u\n", rangingMessage->header.msgLength);
    DEBUG_PRINT("lastTxTimestamps:\n");
    for(int i = 0; i < RANGING_MAX_Tr_UNIT; i++){
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->header.lastTxTimestamps[i].seqNumber, rangingMessage->header.lastTxTimestamps[i].timestamp.full);
    }
    DEBUG_PRINT("Body Units:\n");
    /*
    typedef struct
    {
        uint16_t dest; // 目的地址
        Timestamp_Tuple_t rxTimestamp[RANGING_MAX_RX_UNIT]; // 10 byte * MAX_RX_UNIT
    } __attribute__((packed)) Body_Unit_t;                  // 22
    */
    for(int i = 0; i < RANGING_MAX_BODY_UNIT; i++){
        DEBUG_PRINT("dest: %u\n", rangingMessage->bodyUnits[i].dest);
        DEBUG_PRINT("rxTimestamps:\n");
        for(int j = 0; j < RANGING_MAX_RX_UNIT; j++){
            DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber, rangingMessage->bodyUnits[i].rxTimestamp[j].timestamp.full);
        }
    }
}
// 打印sendBuffer
static void printSendBuffer(){
    DEBUG_PRINT("Send Buffer\n");
    table_index_t index = rangingTableSet.sendBufferTop;
    int count = 0;
    while (index != NULL_INDEX && count < TABLE_BUFFER_SIZE)
    {
        DEBUG_PRINT("seq: %d,Tx: %lld\n", rangingTableSet.sendBuffer[index].seqNumber, rangingTableSet.sendBuffer[index].timestamp.full);
        index = (index - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
        count++;
    }
}

// 打印完整测距表集合
static void printRangingTableSet(){
    DEBUG_PRINT("Ranging Table Set:\n");
    printSendBuffer();
    for (table_index_t i = 0; i < rangingTableSet.size; i++)
    {
        debugPrintRangingTable(&rangingTableSet.neighbor[i]);
    }
}

static Time_t generateRangingMessage(Ranging_Message_t *rangingMessage) {
    // DEBUG_PRINT("generateRangingMessage: Generating ranging message.seq:%d\n",rangingSeqNumber);
    int8_t bodyUnitNumber = 0;
    rangingSeqNumber++;
    int curSeqNumber = rangingSeqNumber;
    //   rangingMessage->header.filter = 0;
    Time_t curTime = xTaskGetTickCount();
    /* Using the default RANGING_PERIOD when DYNAMIC_RANGING_PERIOD is not enabled. */
    Time_t taskDelay = M2T(RANGING_PERIOD);

    /* Generate message body 
    每个消息体包含
    1. 目的地址
    2. 上m次接收时间戳
    */
    int count = 0;
    while(bodyUnitNumber < RANGING_MAX_BODY_UNIT && count < rangingTableSet.size){
        loopIndex = (loopIndex+1) % rangingTableSet.size;
        RangingTable_t *table = &rangingTableSet.neighbor[loopIndex];
        if(table->state == USING){
            DEBUG_PRINT("generateRangingMessage: Generating ranging message for neighbor %u.\n", table->address);
            Body_Unit_t *bodyUnit = &rangingMessage->bodyUnits[bodyUnitNumber];
            bodyUnit->dest = table->address;
            table_index_t p = table->receiveBuffer.head;
            // 从接收缓冲区中取出最近多次接收时间戳
            for(int i = 0; i < RANGING_MAX_RX_UNIT; i++){
                if(p != NULL_INDEX){
                    // 填充对方发送序列号以及接收时间戳
                    DEBUG_PRINT("p:%d",p);
                    bodyUnit->rxTimestamp[i].seqNumber = table->receiveBuffer.tableBuffer[p].remoteSeq;
                    bodyUnit->rxTimestamp[i].timestamp = table->receiveBuffer.tableBuffer[p].Rx;
                    p = table->receiveBuffer.tableBuffer[p].next;
                }
                else{
                    // 数据不足，填充NULL
                    bodyUnit->rxTimestamp[i].seqNumber = NULL_SEQ;
                    bodyUnit->rxTimestamp[i].timestamp.full = NULL_TIMESTAMP;
                }
            }
            bodyUnitNumber++;
        }
        count++;
    }
    /* Generate message header */
    rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
    rangingMessage->header.msgSequence = curSeqNumber;
    // 从发送缓冲区中取出最近多次发送时间戳
    table_index_t sendBufferIndex = rangingTableSet.sendBufferTop;
    for(int i = 0; i < RANGING_MAX_Tr_UNIT; i++){
        if(rangingTableSet.sendBuffer[sendBufferIndex].timestamp.full != NULL_TIMESTAMP){
            // 按序填充发送序列号以及发送时间戳
            rangingMessage->header.lastTxTimestamps[i].seqNumber = rangingTableSet.sendBuffer[sendBufferIndex].seqNumber;
            rangingMessage->header.lastTxTimestamps[i].timestamp = rangingTableSet.sendBuffer[sendBufferIndex].timestamp;
            sendBufferIndex = (sendBufferIndex - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
        }
        else{
            // 数据不足，填充NULL
            rangingMessage->header.lastTxTimestamps[i].seqNumber = NULL_SEQ;
            rangingMessage->header.lastTxTimestamps[i].timestamp.full = NULL_TIMESTAMP;
        }
    }
    //  DEBUG_PRINT("generateRangingMessage: ranging message size = %u with %u body units.\n",
    //              rangingMessage->header.msgLength,
    //              bodyUnitNumber
    //  );
    return taskDelay;
}

static void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
    DEBUG_PRINT("processRangingMessage: Received ranging message from %u,seq:%d\n", rangingMessageWithTimestamp->rangingMessage.header.srcAddress,rangingSeqNumber);
    Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
    uint16_t neighborAddress = rangingMessage->header.srcAddress;
    table_index_t neighborIndex = findRangingTable(neighborAddress);

    /* Handle new neighbor */
    if (neighborIndex == NULL_INDEX) {
        neighborIndex = registerRangingTable(neighborAddress);
        if(neighborIndex == NULL_INDEX){
            DEBUG_PRINT("Warning: Failed to register new neighbor.\n");
            return;
        }
    }
    
    RangingTable_t *neighborRangingTable = &rangingTableSet.neighbor[neighborIndex];
    /* 处理消息体内容 
    每个消息体包含
    1. 目的地址
    2. 上m次接收时间戳
    */
    // printfRangingMessage(rangingMessage);
    for (int i = 0; i < RANGING_MAX_BODY_UNIT; i++) {
        if (rangingMessage->bodyUnits[i].dest == MY_UWB_ADDRESS) {
            // 从消息体中取出接收时间戳
            for (int j = RANGING_MAX_RX_UNIT-1; j >= 0 ; j--) {
                // 按序处理接收时间戳，先插入序号小的（生成报文时序号大的在前）
                if (rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber != NULL_SEQ) {
                    // 查找匹配的发送时间戳
                    // printSendBuffer();
                    table_index_t sendBufferIndex = findSendBufferIndex(rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber);
                    DEBUG_PRINT("processRangingMessage: sendBufferIndex:%d\n",sendBufferIndex);
                    if (sendBufferIndex == NULL_INDEX) {
                        DEBUG_PRINT("Warning: Cannot find corresponding Tx timestamp for Rx timestamp while processing ranging message.\n");
                        continue;
                    }
                    // 添加记录到发送缓冲区
                    addRecord(&neighborRangingTable->sendBuffer, rangingTableSet.sendBuffer[sendBufferIndex].timestamp, rangingMessage->bodyUnits[i].rxTimestamp[j].timestamp, NULL_TF, rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber, rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber);
                    // 触发测距事件
                    updateTof(&neighborRangingTable->sendBuffer, &neighborRangingTable->receiveBuffer, sendBufferIndex, mode);
                }
            }
            break;
        }
    }
    /* 处理消息头内容*/
    // 更新上n次发送时间戳
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        table_index_t receiveBufferIndex = findRemoteSeqIndex(&neighborRangingTable->receiveBuffer, rangingMessage->header.lastTxTimestamps[i].seqNumber);
        DEBUG_PRINT("processRangingMessage: receiveBufferIndex:%d\n",receiveBufferIndex);
        if (receiveBufferIndex != NULL_INDEX) {
            // 更新接收缓冲区中的发送时间戳
            neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].Tx = rangingMessage->header.lastTxTimestamps[i].timestamp;
            // 触发测距事件
            updateTof(&neighborRangingTable->receiveBuffer, &neighborRangingTable->sendBuffer, receiveBufferIndex, mode);
        }
        else{
            DEBUG_PRINT("Warning: Cannot find corresponding Rx timestamp for Tx timestamp while processing ranging message.\n");
        }
    }
    // 添加本次接收时间戳,本地测距处理序号加一
    rangingSeqNumber++;
    addRecord(&neighborRangingTable->receiveBuffer, nullTimeStamp, rangingMessageWithTimestamp->rxTime, NULL_TF, rangingSeqNumber, rangingMessage->header.msgSequence);
    DEBUG_PRINT("processRangingMessage finished.\n");
}

static void uwbRangingTxTask(void *parameters) {
    systemWaitStart();

    /* velocity log variable id */

    UWB_Packet_t txPacketCache;
    txPacketCache.header.srcAddress = uwbGetAddress();
    txPacketCache.header.destAddress = UWB_DEST_ANY;
    txPacketCache.header.type = UWB_RANGING_MESSAGE;
    txPacketCache.header.length = 0;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t *) &txPacketCache.payload;

    while (true) {
        xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

        Time_t taskDelay = generateRangingMessage(rangingMessage);
        txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
        uwbSendPacketBlock(&txPacketCache);
        xSemaphoreGive(rangingTableSet.mu);
        vTaskDelay(taskDelay);
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Timestamp_t rxPacketCache;

    while (true) {
        // portMAX_DELAY取最大值,阻塞等待接收队列中有数据
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);

            processRangingMessage(&rxPacketCache);

            xSemaphoreGive(rangingTableSet.mu);
        }
        vTaskDelay(M2T(100));
    }
}

void modifiedRangingRxCallback(void *parameters) {
    // DEBUG_PRINT("rangingRxCallback \n");

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

    dwTime_t rxTime;
    dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
    Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
    rxMessageWithTimestamp.rxTime = rxTime;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
    rxMessageWithTimestamp.rangingMessage = *rangingMessage;

    xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void modifiedRangingTxCallback(void *parameters) {
    UWB_Packet_t *packet = (UWB_Packet_t *) parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;

    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t *) &txTime.raw);
    
    // 添加记录到发送缓冲区
    xSemaphoreTake(rangingTableSet.mu, portMAX_DELAY);
    // DEBUG_PRINT("TXCB,seq:%d,time:%llu\n",rangingMessage->header.msgSequence,txTime.full);
    // printRangingTableSet(&rangingTableSet);
    rangingTableSet.sendBufferTop = (rangingTableSet.sendBufferTop + 1) % TABLE_BUFFER_SIZE;
    rangingTableSet.sendBuffer[rangingTableSet.sendBufferTop].seqNumber = rangingMessage->header.msgSequence;
    rangingTableSet.sendBuffer[rangingTableSet.sendBufferTop].timestamp = txTime;
    xSemaphoreGive(rangingTableSet.mu);
    // DEBUG_PRINT("modifiedRangingTxCallback: sendBufferIndex:%d,seq:%u,time:%llu\n",rangingTableSet.sendBufferTop,timestamp.seqNumber,timestamp.timestamp.full);
}

void modifiedRangingInit() {
    MY_UWB_ADDRESS = uwbGetAddress();
    rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
    DEBUG_PRINT("modifiedRangingInit: Ranging rxQueue created.\n");
    initRangingTableSet(&rangingTableSet);
    // neighborSetInit(&neighborSet);
    // neighborSetEvictionTimer = xTimerCreate("neighborSetEvictionTimer",
    //                                         M2T(NEIGHBOR_SET_HOLD_TIME / 2),
    //                                         pdTRUE,
    //                                         (void *) 0,
    //                                         neighborSetClearExpireTimerCallback);
    // xTimerStart(neighborSetEvictionTimer, M2T(0));
    // rangingTableSetInit(&rangingTableSet);
    // rangingTableSetEvictionTimer = xTimerCreate("rangingTableSetEvictionTimer",
    //                                             M2T(RANGING_TABLE_HOLD_TIME / 2),
    //                                             pdTRUE,
    //                                             (void *) 0,
    //                                             rangingTableSetClearExpireTimerCallback);
    // xTimerStart(rangingTableSetEvictionTimer, M2T(0));
    // TfBufferMutex = xSemaphoreCreateMutex();

    listener.type = UWB_RANGING_MESSAGE;
    listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
    listener.rxCb = modifiedRangingRxCallback;
    listener.txCb = modifiedRangingTxCallback;
    uwbRegisterListener(&listener);

    xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle);
    xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle);
}