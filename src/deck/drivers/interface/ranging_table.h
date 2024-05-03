#ifndef RANGING_TABLE_H
#define RANGING_TABLE_H

#include <stdint.h>
#include "dwTypes.h"
#include "stdbool.h"

#define TABLE_BUFFER_SIZE 10
#define FREE_QUEUE_SIZE TABLE_BUFFER_SIZE

#define NULL_TIMESTAMP 0
#define NULL_TF NULL_TIMESTAMP
#define NULL_SEQ 0
#define NULL_INDEX -1

 // 每秒光速
#define SPEED_OF_LIGHT_PER_SECOND 299702547

#define table_index_t int8_t 

typedef struct
{
    table_index_t freeIndex[FREE_QUEUE_SIZE];
    table_index_t head;
    table_index_t tail;
    table_index_t size;
} FreeQueue;

typedef struct{
    dwTime_t Tx;    // 发送时间戳
    dwTime_t Rx; // 接收时间戳
    int64_t Tf;// 传输时间戳
    uint16_t localSeq;// 本地序号
    uint16_t remoteSeq;// 远程序号
    table_index_t next; // 下一个节点
    table_index_t pre; // 前一个节点
}TableNode_t;

typedef enum{
    CLASSIC_MODE = 0,
    MODIFIED_MODE
}RangingMODE;

typedef struct 
{
    TableNode_t tableBuffer[TABLE_BUFFER_SIZE];
    FreeQueue freeQueue;            // 空闲指针队列
    table_index_t head;                    // -128 ~ 127
    table_index_t tail;                    // -128 ~ 127
}TableLinkedList_t;

// 测距表状态
typedef enum tableState{
    NULL_STATE = 0,
    USING = 1,
}TableState;

typedef struct
{
    TableState state; // 状态
    uint16_t address; // 地址 
    TableLinkedList_t sendBuffer; // 发送缓冲区
    TableLinkedList_t receiveBuffer; // 接收缓冲区
} RangingTable_t;

// 初始化空闲指针队列
void initFreeQueue(FreeQueue *stack);
// 从空闲指针队列中获取一个空闲指针
table_index_t pop(FreeQueue *stack);
// 将一个空闲指针放回空闲指针队列
void push(FreeQueue *stack, table_index_t index);
// 空间指针队列判空
bool isEmpty(FreeQueue *stack);
// 空间指针队列判满
bool isFull(FreeQueue *stack);

// 初始化链表
void initTableLinkedList(TableLinkedList_t *list);
// 添加时间戳记录
void addRecord(TableLinkedList_t *list, dwTime_t Tx, dwTime_t Rx, int64_t Tf, uint16_t localSeq, uint16_t remoteSeq);
// 删除最后一条记录
void deleteLastRecord(TableLinkedList_t *list);
// 查找小于指定序号的最大有效记录下标
table_index_t findMaxSeqIndex(TableLinkedList_t *list, uint16_t localSeq, RangingMODE mode);
// 查找指定本地序号的记录下标
table_index_t findLocalSeqIndex(TableLinkedList_t *list, uint16_t localSeq);
// 查找指定远程序号的记录下标
table_index_t findRemoteSeqIndex(TableLinkedList_t *list, uint16_t remoteSeq);
// 更新Tof记录
bool updateTof(TableLinkedList_t *listA, TableLinkedList_t *listB, table_index_t firstIndex, RangingMODE mode);

// 初始化测距表
void initRangingTable(RangingTable_t *table);
// 启用测距表
void enableRangingTable(RangingTable_t *table, uint16_t address);
// 禁用测距表
void disableRangingTable(RangingTable_t *table);
// 打印测距表
void debugPrintRangingTable(RangingTable_t *table);

#endif