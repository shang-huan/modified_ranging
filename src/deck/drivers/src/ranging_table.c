#include "FreeRTOS.h"
#include "debug.h"

#include "ranging_table.h"
#include "stdbool.h"

#include "adhocdeck.h"

#ifndef RANGING_TABLE_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

// 初始化空闲指针队列
void initFreeQueue(FreeQueue *stack)
{
    stack->size = FREE_QUEUE_SIZE;
    stack->tail = stack->size - 1;
    stack->head = 0;
    for (table_index_t i = 0; i < FREE_QUEUE_SIZE; i++)
    {
        stack->freeIndex[i] = i;
    }
}
// 从空闲指针队列中获取一个空闲指针
table_index_t pop(FreeQueue *stack)
{
    if(isEmpty(stack)){
        return NULL_INDEX;
    }
    table_index_t index = stack->freeIndex[stack->head];
    stack->head = (stack->head + 1) % FREE_QUEUE_SIZE;
    stack->size--;
    return index;
}

// 将一个空闲指针放回空闲指针队列
void push(FreeQueue *stack, table_index_t index)
{
    if(isFull(stack)){
        return;
    }
    stack->tail = (stack->tail + 1) % FREE_QUEUE_SIZE;
    stack->freeIndex[stack->tail] = index;
    stack->size++;
}

// 空闲指针队列判空
bool isEmpty(FreeQueue *stack)
{
    return stack->size == 0;
}
// 空闲指针队列判满
bool isFull(FreeQueue *stack)
{
    return stack->size == FREE_QUEUE_SIZE;
}

// 初始化链表
void initTableLinkedList(TableLinkedList_t *list)
{
    list->head = NULL_INDEX;
    list->tail = NULL_INDEX;
    for (uint8_t i = 0; i < TABLE_BUFFER_SIZE; i++)
    {
        list->tableBuffer[i].Tx.full = NULL_TIMESTAMP;
        list->tableBuffer[i].Rx.full = NULL_TIMESTAMP;
        list->tableBuffer[i].Tf = NULL_TF;
        list->tableBuffer[i].localSeq = NULL_SEQ;
        list->tableBuffer[i].remoteSeq = NULL_SEQ;
        list->tableBuffer[i].next = NULL_INDEX;
        list->tableBuffer[i].pre = NULL_INDEX;
    }
    initFreeQueue(&list->freeQueue);
}
// 添加时间戳记录，若序列号已存在则更新记录
void addRecord(TableLinkedList_t *list, dwTime_t Tx, dwTime_t Rx, int64_t Tf, uint16_t localSeq, uint16_t remoteSeq)
{
    DEBUG_PRINT("Add Record,localSeq: %d,remoteSeq: %d\n", localSeq, remoteSeq);
    if (isEmpty(&list->freeQueue))
    {
        // 如果满了，删除最后一条记录
        deleteLastRecord(list);
    }
    // 查找该序列号是否已记录
    table_index_t index = list->head;
    while (index != NULL_INDEX)
    {
        if (list->tableBuffer[index].localSeq == localSeq)
        {
            // 如果已记录，更新记录
            if (Tx.full != NULL_TIMESTAMP)
            {
                list->tableBuffer[index].Tx = Tx;
            }
            if (Rx.full != NULL_TIMESTAMP)
            {
                list->tableBuffer[index].Rx = Rx;
            }
            if (Tf != NULL_TF)
            {
                list->tableBuffer[index].Tf = Tf;
            }
            return;
        }
        index = list->tableBuffer[index].next;
    }
    
    // 获取一个空闲指针
    index = pop(&list->freeQueue);
    // 添加记录
    list->tableBuffer[index].Tx = Tx;
    list->tableBuffer[index].Rx = Rx;
    list->tableBuffer[index].Tf = Tf;
    list->tableBuffer[index].localSeq = localSeq;
    list->tableBuffer[index].remoteSeq = remoteSeq;
    // 添加到链表
    if (list->head == NULL_INDEX)
    {
        list->head = index;
        list->tail = index;
    }
    else
    {
        list->tableBuffer[list->head].pre = index;
        list->tableBuffer[index].next = list->head;
        list->head = index;
    }
}
// 删除最后一条记录
void deleteLastRecord(TableLinkedList_t *list)
{
    if (list->tail == NULL_INDEX)
    {
        return;
    }
    // 删除尾节点
    table_index_t index = list->tail;
    list->tail = list->tableBuffer[index].pre;
    if (list->tail != NULL_INDEX)
    {
        list->tableBuffer[list->tail].next = NULL_INDEX;
    }
    // 清空数据
    list->tableBuffer[index].Tx.full = NULL_TIMESTAMP;
    list->tableBuffer[index].Rx.full = NULL_TIMESTAMP;
    list->tableBuffer[index].Tf = NULL_TF;
    list->tableBuffer[index].localSeq = NULL_SEQ;
    list->tableBuffer[index].remoteSeq = NULL_SEQ;
    list->tableBuffer[index].next = NULL_INDEX;
    list->tableBuffer[index].pre = NULL_INDEX;
    DEBUG_PRINT("Delete last record,index: %d\n", index);
    // 放回空闲队列
    push(&list->freeQueue, index);
}
// 查找小于指定序号的最大有效记录下标
table_index_t findMaxSeqIndex(TableLinkedList_t *list, uint16_t localSeq, RangingMODE mode)
{
    table_index_t index = list->head;
    table_index_t noTofIndex = NULL_INDEX;
    while (index != NULL_INDEX)
    {
        // 序号小且Rx、Tx数据完整
        if (list->tableBuffer[index].localSeq < localSeq && list->tableBuffer[index].Rx.full != NULL_TIMESTAMP && list->tableBuffer[index].Tx.full != NULL_TIMESTAMP)
        {
            // 传统模式下，直接返回带有完整Tx与Rx数据即可
            if (mode == CLASSIC_MODE)
                return index;
            if (noTofIndex == NULL_INDEX)
            {
                noTofIndex = index;
            }
            // 改进模式下需要Tof数据完整
            if (list->tableBuffer[index].Tf != NULL_TF)
                return index;
        }
        index = list->tableBuffer[index].next;
    }
    // 若没有Tof数据完整的记录，返回最大有效的记录
    return noTofIndex;
}
// 查找指定本地序号的记录下标
table_index_t findLocalSeqIndex(TableLinkedList_t *list, uint16_t localSeq)
{
    table_index_t index = list->head;
    while (index != NULL_INDEX)
    {
        if (list->tableBuffer[index].localSeq == localSeq)
        {
            return index;
        }
        index = list->tableBuffer[index].next;
    }
    return NULL_INDEX;
}
// 查找指定远程序号的记录下标
table_index_t findRemoteSeqIndex(TableLinkedList_t *list, uint16_t remoteSeq){
    table_index_t index = list->head;
    while (index != NULL_INDEX)
    {
        if (list->tableBuffer[index].remoteSeq == remoteSeq)
        {
            return index;
        }
        index = list->tableBuffer[index].next;
    }
    return NULL_INDEX;

}
// 更新Tof记录
bool updateTof(TableLinkedList_t *listA, TableLinkedList_t *listB, table_index_t firstIndex, RangingMODE mode)
{
    // 依次从A、B、A中取出数据
    table_index_t indexA1 = firstIndex;
    if (indexA1 == NULL_INDEX || listA->tableBuffer[indexA1].Tx.full == NULL_TIMESTAMP || listA->tableBuffer[indexA1].Rx.full == NULL_TIMESTAMP || listA->tableBuffer[indexA1].Tf != NULL_TF)
    {
        DEBUG_PRINT("the lastest record in listA is invalid or the record has owned Tof\n");
        return false;
    }
    table_index_t indexB2 = findMaxSeqIndex(listB, listA->tableBuffer[indexA1].localSeq, mode);
    if (indexB2 == NULL_INDEX)
    {
        DEBUG_PRINT("No valid record in listB\n");
        return false;
    }
    table_index_t indexA3 = findMaxSeqIndex(listA, listB->tableBuffer[indexB2].localSeq, mode);
    if (indexA3 == NULL_INDEX)
    {
        DEBUG_PRINT("No valid record in listA\n");
        return false;
    }
    // 计算Tof
    /*
    A3.Tx          <- Ra ->          B2.Rx      <- Da ->      A1.Tx

            A3.Rx  <- Db ->  B2.Tx              <- Rb ->              A1.Rx
    */
    DEBUG_PRINT("indexA1: %d,seq: %d,tx: %llu,rx:%llu\n", indexA1, listA->tableBuffer[indexA1].localSeq, listA->tableBuffer[indexA1].Tx.full, listA->tableBuffer[indexA1].Rx.full);
    DEBUG_PRINT("indexB2: %d,seq: %d,tx: %llu,rx:%llu\n", indexB2, listB->tableBuffer[indexB2].localSeq, listB->tableBuffer[indexB2].Tx.full, listB->tableBuffer[indexB2].Rx.full);
    DEBUG_PRINT("indexA3: %d,seq: %d,tx: %llu,rx:%llu\n", indexA3, listA->tableBuffer[indexA3].localSeq, listA->tableBuffer[indexA3].Tx.full, listA->tableBuffer[indexA3].Rx.full);
    int64_t Ra = (listB->tableBuffer[indexB2].Rx.full - listA->tableBuffer[indexA3].Tx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (listB->tableBuffer[indexB2].Tx.full - listA->tableBuffer[indexA3].Rx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (listA->tableBuffer[indexA1].Rx.full - listB->tableBuffer[indexB2].Tx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (listA->tableBuffer[indexA1].Tx.full - listB->tableBuffer[indexB2].Rx.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    DEBUG_PRINT("Ra: %lld,Db: %lld,Rb: %lld,Da: %lld\n", Ra, Db, Rb, Da);
    if (mode == CLASSIC_MODE || listA->tableBuffer[indexA3].Tf == NULL_TF || listB->tableBuffer[indexB2].Tf == NULL_TF)
    {
        // 经典计算模式下或者Tof数据不足，采用传统Tof计算方法
        DEBUG_PRINT("Tof data is not complete, use traditional method\n");
        int64_t Tof = ((Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db)) / (2 * (Ra + Db + Rb + Da));
        if (listA->tableBuffer[indexA3].Tf == NULL_TF)
        {
            listA->tableBuffer[indexA3].Tf = Tof;
        }
        if (listB->tableBuffer[indexB2].Tf == NULL_TF)
        {
            listB->tableBuffer[indexB2].Tf = Tof;
        }
        listA->tableBuffer[indexA1].Tf = Tof;
        DEBUG_PRINT("Tof: %lld,d: %f\n", Tof, Tof * 0.4691763978616);
        return false;
    }
    else
    {
        // Tof数据完整，计算全新方式计算Tof
        int64_t Tof = ((Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db)) - (Rb + Db) * listA->tableBuffer[indexA3].Tf - (Rb + Db + Ra + Da) * listB->tableBuffer[indexB2].Tf;
        Tof = Tof / (Ra + Da);
        listA->tableBuffer[indexA1].Tf = Tof;
        DEBUG_PRINT("Tof: %lld,d: %f\n", Tof, Tof * 0.4691763978616);
        int64_t testTof = ((Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db)) / (2 * (Ra + Db + Rb + Da));
        DEBUG_PRINT("testTof: %lld,d: %f\n", testTof, testTof * 0.4691763978616);
        return true;
    }
}

// 初始化测距表
void initRangingTable(RangingTable_t *table)
{
    table->state = NULL_STATE;
    table->address = 0;
    initTableLinkedList(&table->sendBuffer);
    initTableLinkedList(&table->receiveBuffer);
}
// 启用测距表
void enableRangingTable(RangingTable_t *table, uint16_t address)
{
    table->state = USING;
    table->address = address;
    // initRangingTable(table);
}
// 禁用测距表
void disableRangingTable(RangingTable_t *table)
{
    table->state = NULL_STATE;
    table->address = 0;
    initRangingTable(table);
}
// 打印测距表
void debugPrintRangingTable(RangingTable_t *table)
{
    DEBUG_PRINT("Ranging Table State: %s, Address: %d\n", (table->state == NULL_STATE) ? "NOT USING" : "USING", table->address);
    if (table->state == USING)
    {
        DEBUG_PRINT("Send Buffer:\n");
        table_index_t index = table->sendBuffer.head;
        while (index != NULL_INDEX)
        {
            DEBUG_PRINT("localSeq: %d,remoteSeq: %d,Tx: %lld,Rx: %lld,Tf: %lld\n", table->sendBuffer.tableBuffer[index].localSeq, table->sendBuffer.tableBuffer[index].remoteSeq, table->sendBuffer.tableBuffer[index].Tx.full, table->sendBuffer.tableBuffer[index].Rx.full, table->sendBuffer.tableBuffer[index].Tf);
            index = table->sendBuffer.tableBuffer[index].next;
        }
        DEBUG_PRINT("Receive Buffer:\n");
        index = table->receiveBuffer.head;
        while (index != NULL_INDEX)
        {
            DEBUG_PRINT("localSeq: %d,remoteSeq: %d,Tx: %lld,Rx: %lld,Tf: %lld\n", table->receiveBuffer.tableBuffer[index].localSeq, table->receiveBuffer.tableBuffer[index].remoteSeq, table->receiveBuffer.tableBuffer[index].Tx.full, table->receiveBuffer.tableBuffer[index].Rx.full, table->receiveBuffer.tableBuffer[index].Tf);
            index = table->receiveBuffer.tableBuffer[index].next;
        }
    }
}