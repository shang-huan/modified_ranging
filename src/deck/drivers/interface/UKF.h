#ifndef __UKF_H__
#define __UKF_H__
#include "semphr.h"

#include "math.h"
#include "UKFstructure.h"
#include "UnscentedTransform.h"
#include "UKFconfig.h"

extern uint16_t UKFBufferId;

typedef struct 
{
    Coordinate* coordinate;
    double d;
}
MeasurementBufferNode_t;

typedef struct
{
    SemaphoreHandle_t mutex;
    Coordinate* coordinate;
    Velocity* velocity;
    Matrix_t* covX;
    uint16_t measurementBufferLen;
    MeasurementBufferNode_t measurementBuffer[N_MEAS]; //缓存测距信息
}UKFBufferNode_t;

//卡尔曼滤波初始化
void UKFInit();

uint16_t GetUKFBufferId();

void UKFBufferNodeInit(UKFBufferNode_t* bufferNode);
void UKFBufferArrayInit(UKFBufferNode_t* bufferArray, uint16_t length);
bool addUKFBufferMeasurementRecord_FixedPosition(int positionId,double d,uint16_t bufferId);
bool addUKFBufferMeasurementRecord(double x,double y,double z,double d,uint16_t bufferId);

void getCurrentCoordinate(Coordinate* result);//获取当前坐标

void UKFRelativePositionInit();
#endif