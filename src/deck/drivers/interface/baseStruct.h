#ifndef BASE_STRUCT_H
#define BASE_STRUCT_H

#include "modified_ranging_config.h"
#include "adhocdeck.h"

/*
    该文件用于临时解决文件间的循环引用问题
*/

#define TABLE_BUFFER_SIZE 10
#define FREE_QUEUE_SIZE TABLE_BUFFER_SIZE

typedef struct 
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
}__attribute__((packed)) Coordinate16_Tuple_t;

#define table_index_t int8_t 

typedef struct
{
    table_index_t freeIndex[FREE_QUEUE_SIZE];
    table_index_t head;
    table_index_t tail;
    table_index_t size;
} FreeQueue;

typedef enum{
    CLASSIC_MODE = 0,
    MODIFIED_MODE
}RangingMODE;

typedef struct{
    dwTime_t Tx;    // 发送时间戳
    dwTime_t Rx; // 接收时间戳
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    Coordinate16_Tuple_t TxCoordinate; // 发送坐标
    Coordinate16_Tuple_t RxCoordinate; // 接收坐标
    #endif
    int64_t Tf;// 传输时间戳 
    uint16_t localSeq;// 本地序号
    uint16_t remoteSeq;// 远程序号
    table_index_t next; // 下一个节点
    table_index_t pre; // 前一个节点
    #ifdef UKF_RELATIVE_POSITION_ENABLE
    uint16_t ukfBufferId;// UKF缓存ID
    #endif
}__attribute__((packed)) TableNode_t;

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

#endif