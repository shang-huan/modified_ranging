#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "param.h"

#include "debug.h"
#include "log.h"

#include "adhocdeck.h"
#include "timers.h"
#include "static_mem.h"

#include "modified_ranging.h"
#include "ranging_table.h"
#include "nullValue.h"
#include "ranging_buffer.h"

#include "UKF.h"

#ifndef UWB_MODIFIED_RANGING_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

static QueueHandle_t rxRangingQueue;
// static QueueHandle_t rxAmendQueue;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
static TaskHandle_t uwbAmendTxTaskHandle = 0;
static TaskHandle_t uwbAmendRxTaskHandle = 0;

static uint16_t MY_UWB_ADDRESS; // 本机地址
static bool amend_mode = false ; // 是否为修正模式
static RangingMODE ranging_mode = CLASSIC_MODE; // 测距模式

static RangingTableSet_t* rangingTableSet; // 测距表集合
static int loopIndex = 0; // 遍历邻居测距表下标

static SemaphoreHandle_t rangingSeqNumberMutex = NULL; // 测距序号互斥量,防止多线程同时修改
static uint16_t rangingSeqNumber = 1; // 本地测距处理序号
static uint16_t rangingSendNumber = 1; // 本地发送序号

static float viconFX;
static float viconFY;
static float viconFZ;
static uint64_t viconXYZ;

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
    rangingTableSet = (RangingTableSet_t*)malloc(sizeof(RangingTableSet_t));
    rangingTableSet->size = 0;
    rangingTableSet->mu = xSemaphoreCreateMutex();
    rangingTableSet->sendBufferTop = -1;
    for (table_index_t i = 0; i < TABLE_BUFFER_SIZE; i++)
    {
        rangingTableSet->sendBuffer[i].timestamp.full = NULL_TIMESTAMP;
        rangingTableSet->sendBuffer[i].seqNumber = NULL_SEQ;
        #ifdef UKF_RELATIVE_POSITION_ENABLE
        rangingTableSet->ukfBufferId_Send[i] = NULL_INDEX;
        #endif
        #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        rangingTableSet->sendCoordinateBuffer[i] = nullCoordinate;
        #endif
    }
    for (int i = 0; i < MAX_NEIGHBOR_NUM; i++)
    {
        initRangingTable(&rangingTableSet->neighbor[i]);
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
    if(rangingTableSet->size < MAX_NEIGHBOR_NUM){
        enableRangingTable(&rangingTableSet->neighbor[rangingTableSet->size], address);
        index = rangingTableSet->size;
        rangingTableSet->size++;
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
    for(int i = index; i < rangingTableSet->size - 1; i++){
        memcpy(&rangingTableSet->neighbor[i], &rangingTableSet->neighbor[i + 1], sizeof(RangingTable_t));
    }
    disableRangingTable(&rangingTableSet->neighbor[rangingTableSet->size - 1]);
    rangingTableSet->size--;
}

// 查找指定邻居测距表下标
table_index_t findRangingTable(uint16_t address){
    for (table_index_t i = 0; i < rangingTableSet->size; i++)
    {
        if(rangingTableSet->neighbor[i].address == address){
            return i;
        }
    }
    return NULL_INDEX;
}

// 查找指定序号发送缓冲区下标
table_index_t findSendBufferIndex(uint16_t seq){
    table_index_t index = rangingTableSet->sendBufferTop;
    int count = 0;
    // while (rangingTableSet->sendBuffer[index].seqNumber != NULL_SEQ && count < rangingTableSet->size)
    // {
    //     if(rangingTableSet->sendBuffer[index].seqNumber == seq){
    //         return index;
    //     }
    //     index = (index - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
    //     count++;
    // }
    while (rangingTableSet->sendBufferSendId[index] != NULL_SEQ && count < rangingTableSet->size)
    {
        if(rangingTableSet->sendBufferSendId[index] == seq){
            return index;
        }
        index = (index - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
        count++;
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
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu",
        rangingMessage->header.lastTxTimestamps[i].seqNumber, rangingMessage->header.lastTxTimestamps[i].timestamp.full);
        #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
            DEBUG_PRINT(", coordinate:(%u,%u,%u)\n",rangingMessage->header.lastTxCoodinate[i].x, rangingMessage->header.lastTxCoodinate[i].y, rangingMessage->header.lastTxCoodinate[i].z);
        #else
            DEBUG_PRINT("\n");
        #endif
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
            DEBUG_PRINT("seqNumber: %u, timestamp: %llu",
            rangingMessage->bodyUnits[i].rxTimestamp[j].seqNumber, rangingMessage->bodyUnits[i].rxTimestamp[j].timestamp.full);
            #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
            DEBUG_PRINT(", coordinate:(%u,%u,%u)\n",rangingMessage->bodyUnits[i].rxCoodinate[j].x, rangingMessage->bodyUnits[i].rxCoodinate[j].y, rangingMessage->bodyUnits[i].rxCoodinate[j].z);
            #else
                DEBUG_PRINT("\n");
            #endif
        }
    }
}
// 打印sendBuffer
static void printSendBuffer(){
    DEBUG_PRINT("Send Buffer\n");
    table_index_t index = rangingTableSet->sendBufferTop;
    int count = 0;
    while (index != NULL_INDEX && count < TABLE_BUFFER_SIZE)
    {
        DEBUG_PRINT("seq: %d,Tx: %lld", 
        rangingTableSet->sendBuffer[index].seqNumber, rangingTableSet->sendBuffer[index].timestamp.full);
        #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        DEBUG_PRINT(", coordinate:(%u,%u,%u)\n",rangingTableSet->sendCoordinateBuffer[index].x, rangingTableSet->sendCoordinateBuffer[index].y, rangingTableSet->sendCoordinateBuffer[index].z);
        #else
            DEBUG_PRINT("\n");
        #endif
        index = (index - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
        count++;
    }
}

// 打印完整测距表集合
static void printRangingTableSet(){
    DEBUG_PRINT("Ranging Table Set:\n");
    printSendBuffer();
    for (table_index_t i = 0; i < rangingTableSet->size; i++)
    {
        debugPrintRangingTable(&rangingTableSet->neighbor[i]);
    }
}

static Time_t generateRangingMessage(Ranging_Message_t *rangingMessage) {
    // DEBUG_PRINT("generateRangingMessage: Generating ranging message.seq:%d\n",rangingSeqNumber);
    int8_t bodyUnitNumber = 0;
    
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
    while(bodyUnitNumber < RANGING_MAX_BODY_UNIT && count < rangingTableSet->size){
        loopIndex = (loopIndex+1) % rangingTableSet->size;
        RangingTable_t *table = &rangingTableSet->neighbor[loopIndex];
        if(table->state == USING){
            // DEBUG_PRINT("generateRangingMessage: Generating ranging message for neighbor %u.\n", table->address);
            Body_Unit_t *bodyUnit = &rangingMessage->bodyUnits[bodyUnitNumber];
            bodyUnit->dest = table->address;
            table_index_t p = table->receiveBuffer.head;
            // 从接收缓冲区中取出最近多次接收时间戳
            for(int i = 0; i < RANGING_MAX_RX_UNIT; i++){
                if(p != NULL_INDEX){
                    // 填充对方发送序列号以及接收时间戳
                    // DEBUG_PRINT("p:%d",p);
                    bodyUnit->rxTimestamp[i].seqNumber = table->receiveBuffer.tableBuffer[p].remoteSeq;
                    bodyUnit->rxTimestamp[i].timestamp = table->receiveBuffer.tableBuffer[p].Rx;
                    // 填充坐标
                    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
                    bodyUnit->rxCoodinate[i].x = table->receiveBuffer.tableBuffer[p].RxCoordinate.x;
                    bodyUnit->rxCoodinate[i].y = table->receiveBuffer.tableBuffer[p].RxCoordinate.y;
                    bodyUnit->rxCoodinate[i].z = table->receiveBuffer.tableBuffer[p].RxCoordinate.z;
                    #endif
                    p = table->receiveBuffer.tableBuffer[p].next;
                }
                else{
                    // 数据不足，填充NULL
                    bodyUnit->rxTimestamp[i].seqNumber = NULL_SEQ;
                    bodyUnit->rxTimestamp[i].timestamp.full = NULL_TIMESTAMP;
                    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
                    bodyUnit->rxCoodinate[i] = nullCoordinate;
                    #endif
                }
            }
            bodyUnitNumber++;
        }
        count++;
    }
    /* Generate message header */
    
    // 从发送缓冲区中取出最近多次发送时间戳
    table_index_t sendBufferIndex = rangingTableSet->sendBufferTop;
    for(int i = 0; i < RANGING_MAX_Tr_UNIT; i++){
        if(rangingTableSet->sendBuffer[sendBufferIndex].timestamp.full != NULL_TIMESTAMP){
            // 按序填充发送序列号以及发送时间戳
            // rangingMessage->header.lastTxTimestamps[i].seqNumber = rangingTableSet->sendBuffer[sendBufferIndex].seqNumber;
            rangingMessage->header.lastTxTimestamps[i].seqNumber = rangingTableSet->sendBufferSendId[sendBufferIndex];
            rangingMessage->header.lastTxTimestamps[i].timestamp = rangingTableSet->sendBuffer[sendBufferIndex].timestamp;
            // 填充坐标
            #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
            rangingMessage->header.lastTxCoodinate[i].x = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].x;
            rangingMessage->header.lastTxCoodinate[i].y = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].y;
            rangingMessage->header.lastTxCoodinate[i].z = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].z;
            #endif

            sendBufferIndex = (sendBufferIndex - 1 + TABLE_BUFFER_SIZE) % TABLE_BUFFER_SIZE;
        }
        else{
            // 数据不足，填充NULL
            rangingMessage->header.lastTxTimestamps[i].seqNumber = NULL_SEQ;
            rangingMessage->header.lastTxTimestamps[i].timestamp.full = NULL_TIMESTAMP;
            #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
            rangingMessage->header.lastTxCoodinate[i] = nullCoordinate;
            #endif
        }
    }

    rangingMessage->header.packetType = RANGING_PACKET;
    rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
    rangingMessage->header.msgSequence = rangingSendNumber;
    // DEBUG_PRINT("generateRangingMessage: ranging message size = %u with %u body units.\n",
    //             rangingMessage->header.msgLength,
    //             bodyUnitNumber
    // );
    return taskDelay;
}

static void processRangingMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;
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

    RangingTable_t *neighborRangingTable = &rangingTableSet->neighbor[neighborIndex];

    uint16_t curSeq = rangingMessageWithAdditionalInfo->rxSeqNumber;

    TableNode_t node;
    node.Tx = nullTimeStamp;
    node.Rx = rangingMessageWithAdditionalInfo->rxTime;
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    node.TxCoordinate = nullCoordinate;
    node.RxCoordinate.x = rangingMessageWithAdditionalInfo->rxCoordinate.x;
    node.RxCoordinate.y = rangingMessageWithAdditionalInfo->rxCoordinate.y;
    node.RxCoordinate.z = rangingMessageWithAdditionalInfo->rxCoordinate.z;
    #endif
    node.Tf = NULL_TF;
    node.localSeq = curSeq;
    node.remoteSeq = rangingMessage->header.msgSequence;
    addRecord(&neighborRangingTable->receiveBuffer, &node);
    
    // 处理报文内容
    /* 处理消息头内容*/
    // 更新上n次发送时间戳
    for (int i = 0; i < RANGING_MAX_Tr_UNIT; i++) {
        table_index_t receiveBufferIndex = findRemoteSeqIndex(&neighborRangingTable->receiveBuffer, rangingMessage->header.lastTxTimestamps[i].seqNumber);
        // DEBUG_PRINT("processRangingMessage: receiveBufferIndex:%d\n",receiveBufferIndex);
        if (receiveBufferIndex != NULL_INDEX) {
            // 更新接收缓冲区中的发送时间戳
            neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].Tx = rangingMessage->header.lastTxTimestamps[i].timestamp;

            // 打印时间戳信息
            // DEBUG_PRINT("[Rx] localSeq:%u,remoteSeq:%u,TX:%llu,RX:%llu\n",
            //     neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].localSeq,neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].remoteSeq,
            //     neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].Tx.full,neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].Rx.full);

            // 更新发送坐标
            #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
            neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].TxCoordinate.x = rangingMessage->header.lastTxCoodinate[i].x;
            neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].TxCoordinate.y = rangingMessage->header.lastTxCoodinate[i].y;
            neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].TxCoordinate.z = rangingMessage->header.lastTxCoodinate[i].z;
            #endif
            // 触发测距事件
            // updateTof(&neighborRangingTable->receiveBuffer, &neighborRangingTable->sendBuffer, receiveBufferIndex, ranging_mode);
            if(neighborRangingTable->rangingBuffer.sendLength < MAX_RANGING_BUFFER_SIZE){
                firstRecordBuffer(&neighborRangingTable->receiveBuffer, &neighborRangingTable->sendBuffer, receiveBufferIndex, &neighborRangingTable->rangingBuffer, RECEIVER);
                if(neighborRangingTable->rangingBuffer.sendLength == MAX_RANGING_BUFFER_SIZE){
                    // 更新Tof值
                    int64_t initTofMean = getInitTofSum() / MAX_RANGING_BUFFER_SIZE;
                    for (int i = 0; i < neighborRangingTable->rangingBuffer.sendLength; i++)
                    {
                        neighborRangingTable->rangingBuffer.sendBuffer[i].sumTof = initTofMean*2;
                        neighborRangingTable->rangingBuffer.sendBuffer[i].T1 = initTofMean;
                        neighborRangingTable->rangingBuffer.sendBuffer[i].T2 = initTofMean;
                    }
                    DEBUG_PRINT("initTofMean:%lld\n",initTofMean);
                }
            }
            else{
                double d = calculateTof(&neighborRangingTable->rangingBuffer, &neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex],
                    neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].localSeq, RECEIVER, true);
                if(d == -1){
                    DEBUG_PRINT("Warning: Failed to calculate TOF.\n");
                }else{
                    #ifdef UKF_RELATIVE_POSITION_ENABLE
                        addUKFBufferMeasurementRecord_FixedPosition(neighborAddress,d,neighborRangingTable->receiveBuffer.tableBuffer[receiveBufferIndex].ukfBufferId);
                    #endif 
                }
                // printRangingBuffer(&neighborRangingTable->rangingBuffer);
            }
        }
        else{
            DEBUG_PRINT("Warning: Cannot find corresponding Rx timestamp for Tx timestamp while processing ranging message.\n");
        }
    }
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
                    // DEBUG_PRINT("processRangingMessage: sendBufferIndex:%d\n",sendBufferIndex);
                    if (sendBufferIndex == NULL_INDEX) {
                        DEBUG_PRINT("Warning: Cannot find corresponding Tx timestamp for Rx timestamp while processing ranging message.\n");
                        continue;
                    }
                    // 添加记录到发送缓冲区
                    TableNode_t node;
                    node.Tx = rangingTableSet->sendBuffer[sendBufferIndex].timestamp;
                    node.Rx = rangingMessage->bodyUnits[i].rxTimestamp[j].timestamp;
                    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
                    node.TxCoordinate.x = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].x;
                    node.TxCoordinate.y = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].y;
                    node.TxCoordinate.z = rangingTableSet->sendCoordinateBuffer[sendBufferIndex].z;

                    node.RxCoordinate.x = rangingMessageWithAdditionalInfo->rangingMessage.bodyUnits[i].rxCoodinate[j].x;
                    node.RxCoordinate.y = rangingMessageWithAdditionalInfo->rangingMessage.bodyUnits[i].rxCoodinate[j].y;
                    node.RxCoordinate.z = rangingMessageWithAdditionalInfo->rangingMessage.bodyUnits[i].rxCoodinate[j].z;
                    #endif
                    node.Tf = NULL_TF;
                    node.localSeq = rangingTableSet->sendBuffer[sendBufferIndex].seqNumber;
                    // 发送缓冲区无远程ID
                    node.remoteSeq = NULL_INDEX;
                    #ifdef UKF_RELATIVE_POSITION_ENABLE                    
                        node.ukfBufferId = rangingTableSet->ukfBufferId_Send[sendBufferIndex];
                    #endif

                    //打印时间戳信息
                    // DEBUG_PRINT("[Tx] localSeq:%u,remoteSeq:%u,TX:%llu,RX:%llu\n",
                    //     node.localSeq,node.remoteSeq,node.Tx.full,node.Rx.full);

                    table_index_t insertIndex = addRecord(&neighborRangingTable->sendBuffer, &node);
                    // DEBUG_PRINT("processRangingMessage: insertIndex:%d\n",insertIndex);
                    // 触发测距事件
                    // updateTof(&neighborRangingTable->sendBuffer, &neighborRangingTable->receiveBuffer, sendBufferIndex, ranging_mode);
                    if(neighborRangingTable->rangingBuffer.receiveLength < MAX_RANGING_BUFFER_SIZE){
                        firstRecordBuffer(&neighborRangingTable->sendBuffer, &neighborRangingTable->receiveBuffer, insertIndex, &neighborRangingTable->rangingBuffer, SENDER);
                        if(neighborRangingTable->rangingBuffer.receiveLength == MAX_RANGING_BUFFER_SIZE){
                            // 更新Tof值
                            int64_t initTofMean = getInitTofSum() / MAX_RANGING_BUFFER_SIZE;
                            for (int i = 0; i < neighborRangingTable->rangingBuffer.receiveLength; i++)
                            {
                                neighborRangingTable->rangingBuffer.receiveBuffer[i].sumTof = initTofMean*2;
                                neighborRangingTable->rangingBuffer.receiveBuffer[i].T1 = initTofMean;
                                neighborRangingTable->rangingBuffer.receiveBuffer[i].T2 = initTofMean;
                            }
                            DEBUG_PRINT("initTofMean:%lld\n",initTofMean);
                        }
                    }else{
                        double d = calculateTof(&neighborRangingTable->rangingBuffer, &neighborRangingTable->sendBuffer.tableBuffer[insertIndex],
                            neighborRangingTable->sendBuffer.tableBuffer[insertIndex].localSeq, SENDER, true);
                        // printRangingBuffer(&neighborRangingTable->rangingBuffer);
                        if(d == -1){
                            DEBUG_PRINT("Warning: Failed to calculate TOF.\n");
                        }else{
                            #ifdef UKF_RELATIVE_POSITION_ENABLE
                                addUKFBufferMeasurementRecord_FixedPosition(neighborAddress,d,neighborRangingTable->sendBuffer.tableBuffer[insertIndex].ukfBufferId);
                            #endif
                        }
                    }
                }
            }
            break;
        }
    }
    // DEBUG_PRINT("processRangingMessage finished.\n");
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
        xSemaphoreTake(rangingTableSet->mu, portMAX_DELAY);

        Time_t taskDelay = generateRangingMessage(rangingMessage);
        txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
        uwbSendPacketBlock(&txPacketCache);

        xSemaphoreGive(rangingTableSet->mu);
        vTaskDelay(taskDelay);
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Additional_Info_t rxPacketCache;

    while (true) {
        // portMAX_DELAY取最大值,阻塞等待接收队列中有数据
        if (xQueueReceive(rxRangingQueue, &rxPacketCache, portMAX_DELAY)) {
            xSemaphoreTake(rangingTableSet->mu, portMAX_DELAY);

            processRangingMessage(&rxPacketCache);

            xSemaphoreGive(rangingTableSet->mu);
        }
        vTaskDelay(M2T(100));
    }
}

// static void uwbAmendTxTask(void *parameters) {
//     systemWaitStart();

//     /* velocity log variable id */

//     UWB_Packet_t txPacketCache;
//     txPacketCache.header.srcAddress = uwbGetAddress();
//     txPacketCache.header.destAddress = UWB_DEST_ANY;
//     txPacketCache.header.type = UWB_RANGING_MESSAGE;
//     txPacketCache.header.length = 0;
//     Amend_Message_t *amendMessage = (Amend_Message_t *) &txPacketCache.payload;

//     Time_t taskDelay = M2T(100);
//     while (true) {
//         if(amend_mode == true){
//             xSemaphoreTake(rangingTableSet->mu, portMAX_DELAY);

//             //todo 修正测距报文生成
//             //taskDelay  = generateRangingMessage(rangingMessage);

//             txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + amendMessage->header.msgLength;
//             uwbSendPacketBlock(&txPacketCache);
//             xSemaphoreGive(rangingTableSet->mu);
//         }
//         vTaskDelay(taskDelay);
//     }
// }

// static void uwbAmendRxTask(void *parameters) {
//     systemWaitStart();

//     Amend_Message_With_Additional_Info_t rxPacketCache;

//     while (true) {
//         // portMAX_DELAY取最大值,阻塞等待接收队列中有数据
//         if (xQueueReceive(rxAmendQueue, &rxPacketCache, portMAX_DELAY)) {
//             xSemaphoreTake(rangingTableSet->mu, portMAX_DELAY);

//             // todo: processAmendMessage(&rxPacketCache);

//             xSemaphoreGive(rangingTableSet->mu);
//         }
//         vTaskDelay(M2T(100));
//     }
// }

void modifiedRangingRxCallback(void *parameters) {
    // DEBUG_PRINT("rangingRxCallback \n");

    dwTime_t rxTime;
    dwt_readrxtimestamp((uint8_t *) &rxTime.raw);

    xSemaphoreTake(rangingSeqNumberMutex, portMAX_DELAY);
    rangingSeqNumber++;
    uint16_t curSeq = rangingSeqNumber;
    xSemaphoreGive(rangingSeqNumberMutex);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Packet_t *packet = (UWB_Packet_t *) parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;

    if(rangingMessage->header.packetType == RANGING_PACKET){ 
        Ranging_Message_With_Additional_Info_t rxMessageWithAdditionalInfo;
        rxMessageWithAdditionalInfo.rxTime = rxTime;
        rxMessageWithAdditionalInfo.rxSeqNumber = curSeq;
        #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        uint64_t curViconXYZ = viconXYZ;
        uint64_t viconX = (curViconXYZ & 0x0000FFFF00000000) >> 32;
        uint64_t viconY = (curViconXYZ & 0x00000000FFFF0000) >> 16;
        uint64_t viconZ = (curViconXYZ & 0x000000000000FFFF);
        uint16_t x = (uint16_t)viconX;
        uint16_t y = (uint16_t)viconY;
        uint16_t z = (uint16_t)viconZ;
        rxMessageWithAdditionalInfo.rxCoordinate.x = x;
        rxMessageWithAdditionalInfo.rxCoordinate.y = y;
        rxMessageWithAdditionalInfo.rxCoordinate.z = z;
        // DEBUG_PRINT("modifiedRangingRxCallback: coordinate:(%u,%u,%u)\n",rxMessageWithAdditionalInfo.rxCoordinate.x,
        //     rxMessageWithAdditionalInfo.rxCoordinate.y, rxMessageWithAdditionalInfo.rxCoordinate.z);
        #endif
        rxMessageWithAdditionalInfo.rangingMessage = *rangingMessage;
        xQueueSendFromISR(rxRangingQueue, &rxMessageWithAdditionalInfo, &xHigherPriorityTaskWoken);
    }
    // else if(rangingMessage->header.packetType == AMEND_PACKET){
    //     Amend_Message_t *amendMessage = (Amend_Message_t *) packet->payload;
    //     Amend_Message_With_Additional_Info_t rxMessageWithAdditionalInfo;
    //     rxMessageWithAdditionalInfo.rxTime = rxTime;
    //     #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    //     rxMessageWithAdditionalInfo.rxCoordinate.x = 100 * logGetFloat(logGetVarId("stateEstimate", "x"));
    //     rxMessageWithAdditionalInfo.rxCoordinate.y = 100 * logGetFloat(logGetVarId("stateEstimate", "y"));
    //     rxMessageWithAdditionalInfo.rxCoordinate.z = 100 * logGetFloat(logGetVarId("stateEstimate", "z"));
    //     #endif
    //     rxMessageWithAdditionalInfo.rangingMessage = *amendMessage;
    //     xQueueSendFromISR(rxAmendQueue, &rxMessageWithAdditionalInfo, &xHigherPriorityTaskWoken);
    // }
    else{
        DEBUG_PRINT("Warning: Unknown packet type received.\n");
    }
    // DEBUG_PRINT("modifiedRangingRxCallback: rxBufferIndex:%d,seq:%u,time:%llu\n",rangingTableSet->sendBufferTop
    //         ,rangingTableSet->sendBuffer[rangingTableSet->sendBufferTop].seqNumber
    //         ,rangingTableSet->sendBuffer[rangingTableSet->sendBufferTop].timestamp.full);
    // DEBUG_PRINT("modifiedRangingRxCallback: rxBufferIndex:%d,seq:%u,time:%llu\n",rangingTableSet->sendBufferTop
}

void modifiedRangingTxCallback(void *parameters) {

    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t *) &txTime.raw);

    xSemaphoreTake(rangingSeqNumberMutex, portMAX_DELAY);
    rangingSeqNumber++;
    uint16_t curSeq = rangingSeqNumber;
    xSemaphoreGive(rangingSeqNumberMutex);

    // 发送序号增加
    rangingSendNumber++;

    UWB_Packet_t *packet = (UWB_Packet_t *) parameters;
    
    // 添加记录到发送缓冲区
    xSemaphoreTake(rangingTableSet->mu, portMAX_DELAY);
    // DEBUG_PRINT("TXCB,seq:%d,time:%llu\n",rangingMessage->header.msgSequence,txTime.full);
    // printRangingTableSet(&rangingTableSet);
    rangingTableSet->sendBufferTop = (rangingTableSet->sendBufferTop + 1) % TABLE_BUFFER_SIZE;
    int8_t sendBufferIndex = rangingTableSet->sendBufferTop;
    rangingTableSet->sendBuffer[sendBufferIndex].seqNumber = curSeq; 
    rangingTableSet->sendBuffer[sendBufferIndex].timestamp = txTime;
    #ifdef UKF_RELATIVE_POSITION_ENABLE
    rangingTableSet->ukfBufferId_Send[sendBufferIndex] = UKFBufferId;
    #endif
    Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
    if(rangingMessage->header.packetType == RANGING_PACKET){
        // rangingTableSet->sendBuffer[sendBufferIndex].seqNumber = curSeq; 
        rangingTableSet->sendBufferSendId[sendBufferIndex] = rangingMessage->header.msgSequence;
    }
    else if(rangingMessage->header.packetType == AMEND_PACKET){
        Amend_Message_t *amendMessage = (Amend_Message_t *) packet->payload;
        // rangingTableSet->sendBuffer[sendBufferIndex].seqNumber = curSeq;
        rangingTableSet->sendBufferSendId[sendBufferIndex] = amendMessage->header.msgSequence;
    }
    else{
        DEBUG_PRINT("Warning: Unknown packet type received.\n");
    }
    
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    uint64_t curViconXYZ = viconXYZ;
    uint64_t viconX = (curViconXYZ & 0x0000FFFF00000000) >> 32;
    uint64_t viconY = (curViconXYZ & 0x00000000FFFF0000) >> 16;
    uint64_t viconZ = (curViconXYZ & 0x000000000000FFFF);
    uint16_t x = (uint16_t)viconX;
    uint16_t y = (uint16_t)viconY;
    uint16_t z = (uint16_t)viconZ;
    rangingTableSet->sendCoordinateBuffer[sendBufferIndex].x = x;
    rangingTableSet->sendCoordinateBuffer[sendBufferIndex].y = y;
    rangingTableSet->sendCoordinateBuffer[sendBufferIndex].z = z;
    // DEBUG_PRINT("modifiedRangingTxCallback: coordinate:(%u,%u,%u)\n",rangingTableSet->sendCoordinateBuffer[sendBufferIndex].x,
    //     rangingTableSet->sendCoordinateBuffer[sendBufferIndex].y, rangingTableSet->sendCoordinateBuffer[sendBufferIndex].z);
    #endif

    xSemaphoreGive(rangingTableSet->mu);
    // DEBUG_PRINT("modifiedRangingTxCallback: sendBufferIndex:%d,seq:%u,time:%llu\n",sendBufferIndex
    //         ,rangingTableSet->sendBuffer[sendBufferIndex].seqNumber
    //         ,rangingTableSet->sendBuffer[sendBufferIndex].timestamp.full);
}

void modifiedRangingInit() {
    MY_UWB_ADDRESS = uwbGetAddress();
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    viconFX = 11.17914;
    viconFY = 12.84513;
    viconFZ = 33.04726;
    uint64_t viconX = (uint64_t)(roundf(viconFX));
    uint64_t viconY = (uint64_t)(roundf(viconFY));
    uint64_t viconZ = (uint64_t)(roundf(viconFZ));
    viconXYZ = (viconX << 32 | viconY << 16 | viconZ);
    #endif
    rxRangingQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
    // rxAmendQueue = xQueueCreate(AMEND_RX_QUEUE_SIZE, AMEND_RX_QUEUE_SIZE);
    // DEBUG_PRINT("modifiedRangingInit: Ranging rxRangingQueue created.\n");
    initRangingTableSet(&rangingTableSet);
    rangingSeqNumberMutex = xSemaphoreCreateMutex();
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
    listener.rxQueue = NULL; // handle rxRangingQueue in swarm_ranging.c instead of adhocdeck.c
    listener.rxCb = modifiedRangingRxCallback;
    listener.txCb = modifiedRangingTxCallback;
    uwbRegisterListener(&listener);

    xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle);
    xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle);
    
    DEBUG_PRINT("modifiedRangingInit: Ranging txTask and rxTask created.\n");
    // xTaskCreate(uwbAmendTxTask, ADHOC_DECK_AMEND_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
    //             ADHOC_DECK_TASK_PRI, &uwbAmendTxTaskHandle);
    // xTaskCreate(uwbAmendRxTask, ADHOC_DECK_AMEND_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
    //             ADHOC_DECK_TASK_PRI, &uwbAmendRxTaskHandle);
}

PARAM_GROUP_START(vicon)
PARAM_ADD(PARAM_FLOAT, viconX, &viconFX)
PARAM_ADD(PARAM_FLOAT, viconY, &viconFY)
PARAM_ADD(PARAM_FLOAT, viconZ, &viconFZ)
PARAM_ADD(PARAM_8BYTES, viconXYZ, &viconXYZ)
PARAM_GROUP_STOP(vicon)

