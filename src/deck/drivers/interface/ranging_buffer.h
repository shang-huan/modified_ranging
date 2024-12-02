#ifndef RANGE_BUFFER_H
#define RANGE_BUFFER_H

#include "baseStruct.h"
#include "dwTypes.h"

// #define ENABLE_CLASSIC_TOLERANCE
// #define ENABLE_RECORD_COORDINATE

#define MAX_RANGING_BUFFER_SIZE 5

#define CONVERGENCE_THRESHOLD 1 // 收敛阈值

typedef enum
{
    SENDER, // 代表本次通信自身是发送方
    RECEIVER // 代表本次通信自身是接收方
}StatusType;

typedef struct
{
    dwTime_t sendTx;    
    dwTime_t sendRx; 
    dwTime_t receiveTx;   
    dwTime_t receiveRx;
    int64_t T1;
    int64_t T2;
    int64_t sumTof;

    #ifdef ENABLE_RECORD_COORDINATE
        Coordinate16_Tuple_t sendTxCoordinate; 
        Coordinate16_Tuple_t sendRxCoordinate; 
        Coordinate16_Tuple_t receiveTxCoordinate;
        Coordinate16_Tuple_t receiveRxCoordinate;
    #endif

    uint16_t localSeq;
    uint16_t preLocalSeq; //前一个本地序号
}RangingBufferNode; // 一次往返通信有效记录缓存

typedef struct 
{
    uint8_t sendLength;
    uint8_t receiveLength;
    table_index_t topSendBuffer;
    table_index_t topReceiveBuffer;
    RangingBufferNode sendBuffer[MAX_RANGING_BUFFER_SIZE];
    RangingBufferNode receiveBuffer[MAX_RANGING_BUFFER_SIZE];
}RangingBuffer; // 缓存多次有效往返通信记录

void initRangingBuffer(RangingBuffer *buffer);
void initRangingBufferNode(RangingBufferNode *node);
void addRangingBuffer(RangingBuffer *buffer, RangingBufferNode *node, StatusType status);
table_index_t searchRangingBuffer(RangingBuffer *buffer, uint16_t localSeq, StatusType status);
bool calculateTof(RangingBuffer *buffer, dwTime_t Tx, dwTime_t Rx, uint16_t localSeq, uint16_t checkLocalSeq, StatusType status, bool flag);
bool firstRecordBuffer(TableLinkedList_t *listA, TableLinkedList_t *listB, table_index_t firstIndex, RangingBuffer* rangingBuffer, StatusType status);

void printRangingBuffer(RangingBuffer *buffer);
#endif