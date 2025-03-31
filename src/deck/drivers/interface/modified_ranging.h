#ifndef MODIFIED_RANGING_H
#define MODIFIED_RANGING_H

#include <stdint.h>
#include "ranging_table.h"
#include "semphr.h"
#include "adhocdeck.h"
#include "modified_ranging_config.h"

#define MAX_NEIGHBOR_NUM 10

/* Ranging Constants */
#define RANGING_PERIOD 500     // default in 200ms
#define RANGING_PERIOD_MIN 50  // default 50ms
#define RANGING_PERIOD_MAX 500 // default 500ms

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 5
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Additional_Info_t)
#define AMEND_RX_QUEUE_SIZE 5
#define AMEND_RX_QUEUE_SIZE sizeof(Amend_Message_With_Additional_Info_t)

/* Ranging Struct Constants */
#define RANGING_MESSAGE_SIZE_MAX UWB_PAYLOAD_SIZE_MAX
#define RANGING_MESSAGE_PAYLOAD_SIZE_MAX (RANGING_MESSAGE_SIZE_MAX - sizeof(Ranging_Message_Header_t))
#define RANGING_MAX_RX_UNIT 1
#define RANGING_MAX_Tr_UNIT 1
#define RANGING_MAX_BODY_UNIT (RANGING_MESSAGE_PAYLOAD_SIZE_MAX / sizeof(Body_Unit_t))

// #define RANGING_TABLE_HOLD_TIME (6 * RANGING_PERIOD_MAX)
// #define Tr_Rr_BUFFER_POOL_SIZE 3
// #define Tf_BUFFER_POOL_SIZE (2 * RANGING_PERIOD_MAX / RANGING_PERIOD_MIN)

typedef enum packetType
{
    RANGING_PACKET = 0,
    AMEND_PACKET = 1
} PacketType;

/* Timestamp Tuple */
typedef struct
{
    dwTime_t timestamp;                      
    uint16_t seqNumber;                      
} __attribute__((packed)) Timestamp_Tuple_t; 

/* Body Unit */
typedef struct
{
    //   struct {
    //     uint8_t MPR: 1;
    //     uint8_t RESERVED: 7;
    //   } flags; // 1 byte //暂时不知作用
    uint16_t dest;                                      
    Timestamp_Tuple_t rxTimestamp[RANGING_MAX_RX_UNIT]; 
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        Coordinate16_Tuple_t rxCoodinate[RANGING_MAX_RX_UNIT];
    #endif
} __attribute__((packed)) Body_Unit_t;                  

/* Ranging Message Header*/
typedef struct
{
    PacketType packetType;                                   
    uint16_t srcAddress;                                     
    uint16_t msgSequence;                                    
    Timestamp_Tuple_t lastTxTimestamps[RANGING_MAX_Tr_UNIT];
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        Coordinate16_Tuple_t lastTxCoodinate[RANGING_MAX_Tr_UNIT];
    #endif
    uint16_t msgLength;                                      
    //   uint16_t filter; // 16 bits bloom filter //暂时不知作用
} __attribute__((packed)) Ranging_Message_Header_t; // 36

/* Ranging Message */
/* 最终携带内容：
    1. 源地址
    2. 消息序列号
    3. 上n次发送时间戳
    4. 上n次发送坐标
    5. 消息长度
    6. 消息体数组
每个消息体包含
    1. 目的地址
    2. 上m次接收时间戳
    3. 上m次接收坐标
*/
typedef struct
{
    Ranging_Message_Header_t header;
    Body_Unit_t bodyUnits[RANGING_MAX_BODY_UNIT];
} __attribute__((packed)) Ranging_Message_t; 

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct
{
    Ranging_Message_t rangingMessage;
    dwTime_t rxTime;
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        Coordinate16_Tuple_t rxCoordinate;
    #endif
} __attribute__((packed)) Ranging_Message_With_Additional_Info_t;

/* Amend packet 
    用于修正测距，在原有报文结构基础上增加令牌环机制
*/
typedef struct
{
    PacketType packetType;      // 报文类型
    uint16_t srcAddress;         // 源地址               
    uint16_t msgSequence;       // 消息序列号                                 
    uint16_t token;             // 令牌
    Timestamp_Tuple_t lastTxTimestamps[RANGING_MAX_Tr_UNIT];  // 上n次发送时间戳
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        Coordinate16_Tuple_t lastTxCoodinate[RANGING_MAX_Tr_UNIT]; // 上n次发送坐标
    #endif
    uint16_t msgLength;                                      // 消息长度
} __attribute__((packed)) Amend_Message_Header_t; // 36

typedef struct
{
    Amend_Message_Header_t header;  // 修正报文头
    Body_Unit_t bodyUnits[RANGING_MAX_BODY_UNIT]; // 测距消息体
} __attribute__((packed)) Amend_Message_t; // 36 + 22 * MAX_BODY_UNIT

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct
{
    Amend_Message_t rangingMessage;
    dwTime_t rxTime;
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
        Coordinate16_Tuple_t rxCoordinate;
    #endif
} __attribute__((packed)) Amend_Message_With_Additional_Info_t;

typedef struct
{
    int size;                     // 邻居数量
    SemaphoreHandle_t mu;         // 互斥量

    table_index_t sendBufferTop; // 缓冲区头索引
    Timestamp_Tuple_t sendBuffer[TABLE_BUFFER_SIZE]; // 发送缓冲区，用于存储发送消息的时间戳
    #ifdef UKF_RELATIVE_POSITION_ENABLE
    uint16_t ukfBufferId_Send[TABLE_BUFFER_SIZE]; 
    #endif
    #ifdef UWB_COMMUNICATION_SEND_POSITION_ENABLE
    Coordinate16_Tuple_t sendCoordinateBuffer[TABLE_BUFFER_SIZE]; // 发送缓冲区，用于存储发送消息的坐标
    #endif
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