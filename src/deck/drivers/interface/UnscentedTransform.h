#ifndef UNSCENTEDTRANSFORM_H
#define UNSCENTEDTRANSFORM_H

#include "stdlib.h"
#include "UKFstructure.h"
#include "MatrixTool.h"
#include "UKFstructure.h"
#include "UKFconfig.h"

extern double* weightM; // 计算近似均值时权值
extern double* weightC; // 计算近似协方差时权值
extern Coordinate** sigmaPoints; // sigma点
extern Matrix_t* Q; // 状态量噪声Q
extern Matrix_t* R; // 量测量噪声R

// sigam点初始化
void sigmaPointInit();
// 无迹变换权重初始化
void weightInit();
// 噪声初始化()
void noiseInit();
// 无迹变换权重打印
void weightPrint();

// sigma采样
bool sigmaPointsUpdate(Coordinate *X, Matrix_t *P);

// 计算点集均值
bool meanOfPoints(Coordinate *result, Coordinate **points, double* weightM, int n);
// 计算点集协方差矩阵
bool covarianceOfPoints(Matrix_t *result, Coordinate **points, Coordinate *mean, double* weightC, int n);
// bool covarianceOfPointsTest(Matrix_t *result, Coordinate *points, Coordinate *mean, int n);

// 计算量测均值
bool meanOfMeasurements(Measurement *result, Measurement *measurements, double* weightM, int n);
// 计算量测协方差矩阵
bool covarianceOfMeasurements(Matrix_t *result, Measurement *measurements, Measurement *mean, double* weightC, int n);

// 计算sigmaXZ
bool covarianceOfPointsAndMeasurements(Matrix_t *result, Coordinate **points, Coordinate *mean, Measurement *measurements, Measurement *meanZ, double* weightC, int n);
#endif