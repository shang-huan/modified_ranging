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

void UKFBufferNodeInit(UKFBufferNode_t* bufferNode);
void UKFBufferArrayInit(UKFBufferNode_t* bufferArray, uint16_t length);
bool addUKFBufferMeasurementRecord_FixedPosition(int positionId,double d,uint16_t bufferId);
bool addUKFBufferMeasurementRecord(double x,double y,double z,double d,uint16_t bufferId);

double GaussianNormalRand();
double GaussianRand(double mu, double sigma);

bool equationsOfMotion(Coordinate *newP, Coordinate* oldP, Velocity* V);//动力学方程
double equationsOfMeasurement(Coordinate *coordinateA, Coordinate *coordinateB);//量测方程

bool KUpdate(Matrix_t* K, Matrix_t* sigmaXZ, Matrix_t* sigmaZZ);//更新卡尔曼增益
bool stateUpdate(Coordinate* newP, Coordinate* oldP, Matrix_t* K, Measurement *z, Measurement *zMean);//状态更新
bool covarianceUpdate(Matrix_t *sigmaNew, Matrix_t* sigmaP, Matrix_t* K, Matrix_t* sigmaZ);//协方差更新

void getCurrentCoordinate(Coordinate* result);//获取当前坐标

void UKFRelativePositionInit();
#endif