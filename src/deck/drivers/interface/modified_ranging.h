#ifndef MODIFIED_RANGING_H
#define MODIFIED_RANGING_H

#include <stdint.h>
#include "ranging_table.h"
#include "semphr.h"
#include "adhocdeck.h"

// #define UWB_MODIFIED_RANGING_DEBUG_ENABLE

#define MAX_NEIGHBOR_NUM 10

/* Ranging Constants */
#define RANGING_PERIOD 500     // default in 200ms
#define RANGING_PERIOD_MIN 50  // default 50ms
#define RANGING_PERIOD_MAX 500 // default 500ms

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 5
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Struct Constants */
#define RANGING_MESSAGE_SIZE_MAX UWB_PAYLOAD_SIZE_MAX
#define RANGING_MESSAGE_PAYLOAD_SIZE_MAX (RANGING_MESSAGE_SIZE_MAX - sizeof(Ranging_Message_Header_t))
#define RANGING_MAX_RX_UNIT 2
#define RANGING_MAX_Tr_UNIT 3
#define RANGING_MAX_BODY_UNIT (RANGING_MESSAGE_PAYLOAD_SIZE_MAX / sizeof(Body_Unit_t))

// #define RANGING_TABLE_HOLD_TIME (6 * RANGING_PERIOD_MAX)
// #define Tr_Rr_BUFFER_POOL_SIZE 3
// #define Tf_BUFFER_POOL_SIZE (2 * RANGING_PERIOD_MAX / RANGING_PERIOD_MIN)

/* Timestamp Tuple */
typedef struct
{
    dwTime_t timestamp;                      // 8 byte
    uint16_t seqNumber;                      // 2 byte
} __attribute__((packed)) Timestamp_Tuple_t; // 10 byte

/* Body Unit */
typedef struct
{
    //   struct {
    //     uint8_t MPR: 1;
    //     uint8_t RESERVED: 7;
    //   } flags; // 1 byte //暂时不知作用
    uint16_t dest;                                      // 2 byte
    Timestamp_Tuple_t rxTimestamp[RANGING_MAX_RX_UNIT]; // 10 byte * MAX_RX_UNIT
} __attribute__((packed)) Body_Unit_t;                  // 22

/* Ranging Message Header*/
typedef struct
{
    uint16_t srcAddress;                                     // 2 byte
    uint16_t msgSequence;                                    // 2 byte
    Timestamp_Tuple_t lastTxTimestamps[RANGING_MAX_Tr_UNIT]; // 10 byte * MAX_Tr_UNIT
    uint16_t msgLength;                                      // 2 byte
    //   uint16_t filter; // 16 bits bloom filter //暂时不知作用
} __attribute__((packed)) Ranging_Message_Header_t; // 36

/* Ranging Message */
/* 最终携带内容：
    1. 源地址
    2. 消息序列号
    3. 上n次发送时间戳
    4. 消息长度
    5. 消息体数组
每个消息体包含
    1. 目的地址
    2. 上m次接收时间戳
*/
typedef struct
{
    Ranging_Message_Header_t header;
    Body_Unit_t bodyUnits[RANGING_MAX_BODY_UNIT];
} __attribute__((packed)) Ranging_Message_t; // 36 + 22 * MAX_BODY_UNIT

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct
{
    Ranging_Message_t rangingMessage;
    dwTime_t rxTime;
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;

typedef struct
{
    int size;                     // 邻居数量
    SemaphoreHandle_t mu;         // 互斥量

    table_index_t sendBufferTop; // 缓冲区头索引
    Timestamp_Tuple_t sendBuffer[TABLE_BUFFER_SIZE]; // 发送缓冲区，用于存储发送消息的时间戳
    
    RangingTable_t neighbor[MAX_NEIGHBOR_NUM];      // 邻居测距表
} RangingTableSet_t;

// 初始化测距表集
void initRangingTableSet();
// 注册测距表
table_index_t registerRangingTable(uint16_t address);
// 注销测距表
void unregisterRangingTable(uint16_t address);
// 查找指定邻居测距表下标
table_index_t findRangingTable(uint16_t address);
// 查找指定序号发送缓冲区下标
table_index_t findSendBufferIndex(uint16_t seq);

void modifiedRangingInit();
#endif