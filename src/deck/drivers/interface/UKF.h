#ifndef __UKF_H__
#define __UKF_H__
#include "semphr.h"

#include "math.h"
#include "UKFstructure.h"
#include "UnscentedTransform.h"
#include "UKFconfig.h"

#define UKF_TASK_NAME "UKF"
#define UKF_TASK_STACKSIZE 512
#define UKF_TASK_PRI 3

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

void addMeasurementRecord(uint16_t sourceAdr,double d,uint16_t bufferId);
void getCurrentCoordinate(Coordinate* result);//获取当前坐标

void UKFRelativePositionInit();
#endif