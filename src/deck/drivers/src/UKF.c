#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"
#include "string.h"

#include "UKF.h"
#include "stdlib.h"
#include "stdio.h"
#include "MatrixTool.h"
#include "UnscentedTransform.h"
#include "read_write_lock.h"

static TaskHandle_t ukfTaskHandle;
Coordinate* currentCoordinate;
Velocity* currentVelocity;

Coordinate** currentMeasurementPoint;
Measurement* zValue;
Measurement* zMean;
Measurement* zTrue;

Matrix_t *covX;
Matrix_t *covZ;
Matrix_t *covXZ;

Matrix_t *K;

/*
UKFTask 所用数据
*/
uint16_t UKFBufferId;
// 测距缓冲区
MeasurementBufferNode_t* measurementBuffer;
SemaphoreHandle_t measurementBufferMutex; 
bool hasNewMeasurement = false;

// 边缘坐标
uint16_t* edgerAdr;
Coordinate** edgerCoordinate;

uint16_t GetUKFBufferId(){
    return UKFBufferId;
}

// 获取当前速度
void getCurrentVelocity(Velocity* velocity)
{
    // cm/s
    double vx = logGetFloat(logGetVarId("stateEstimate", "vx")) * 100; 
    double vy = logGetFloat(logGetVarId("stateEstimate", "vy")) * 100;
    double vz = logGetFloat(logGetVarId("stateEstimate", "vz")) * 100;
    DEBUG_PRINT("vx = %f, vy = %f, vz = %f\n", vx, vy, vz);
    setVelocity(velocity, vx, vy, vz);
}

// 动力学方程
bool equationsOfMotion(Coordinate *newP, Coordinate *oldP, Velocity *V)
{
    if (newP == NULL)
    {
        DEBUG_PRINT("newP为空，请分配空间\n");
        return false;
    }
    // X = X + VT
    static Matrix_t *temp;
    if(temp == NULL){
        temp = createMatrix_t(newP->coordinateXYZ->row, newP->coordinateXYZ->col);
    }else if(temp->row!= newP->coordinateXYZ->row || temp->col != newP->coordinateXYZ->col){
        // 释放空间
        Matrix_t_delete(temp);
        temp = createMatrix_t(newP->coordinateXYZ->row, newP->coordinateXYZ->col);
    }
    Matrix_t_scalar_multiply(temp, V->velocityXYZ, T);
    Matrix_t_add(newP->coordinateXYZ, oldP->coordinateXYZ, temp);
    return true;
}

// 量测方程
double equationsOfMeasurement(Coordinate *coordinateA, Coordinate *coordinateB)
{
    // 计算两个三维坐标之间的距离
    double res = sqrt(pow(coordinateA->coordinateXYZ->data[0][0] - coordinateB->coordinateXYZ->data[0][0], 2) +
                                      pow(coordinateA->coordinateXYZ->data[1][0] - coordinateB->coordinateXYZ->data[1][0], 2) +
                                      pow(coordinateA->coordinateXYZ->data[2][0] - coordinateB->coordinateXYZ->data[2][0], 2));
    return res;
}

// 更新卡尔曼增益
bool KUpdate(Matrix_t *K, Matrix_t *sigmaXZ, Matrix_t *sigmaZZ)
{
    if (sigmaXZ == NULL || sigmaZZ == NULL)
    {
        DEBUG_PRINT("sigmaXZ或sigmaZZ为空，请分配空间\n");
        return false;
    }
    // K = sigmaXZ * sigmaZZ^-1
    Matrix_t *temp = createMatrix_t(sigmaZZ->row, sigmaZZ->col);
    Matrix_t_inverse(temp, sigmaZZ);
    Matrix_t_multiply(K, sigmaXZ, temp);
    Matrix_t_delete(temp);
    return true;
}

// 状态量更新
bool stateUpdate(Coordinate *newP, Coordinate *oldP, Matrix_t *K, Measurement *z, Measurement *zMean)
{
    if (newP == NULL || oldP == NULL || K == NULL || z == NULL || zMean == NULL)
    {
        DEBUG_PRINT("newP或oldP或K或z或zMean为空，请分配空间\n");
        return false;
    }
    // X = X + K(Z - ZMean)
    Matrix_t *tempZ = createMatrix_t(z->measurement->row, z->measurement->col);
    Matrix_t_subtract(tempZ, z->measurement, zMean->measurement);
    Matrix_t *temp = createMatrix_t(K->row, z->measurement->col);
    Matrix_t_multiply(temp, K, tempZ);
    Matrix_t_add(newP->coordinateXYZ, oldP->coordinateXYZ, temp);
    Matrix_t_delete(temp);
    Matrix_t_delete(tempZ);
    return true;
}

// 协方差更新
bool covarianceUpdate(Matrix_t *sigmaNew, Matrix_t *sigmaP, Matrix_t *K, Matrix_t *sigmaZ)
{
    if (sigmaP == NULL || K == NULL || sigmaZ == NULL)
    {
        DEBUG_PRINT("sigmaP或K或sigmaZ为空，请分配空间\n");
        return false;
    }
    // P = P - K*Z*KT
    Matrix_t *temp = createMatrix_t(K->row, sigmaZ->col);
    Matrix_t_multiply(temp, K, sigmaZ);
    // DEBUG_PRINT("KZ = \n");
    // Matrix_t_print(temp);
    Matrix_t *temp2 = createMatrix_t(K->col, K->row);
    Matrix_t_transpose(temp2, K);
    // DEBUG_PRINT("KT = \n");
    // Matrix_t_print(temp2);
    Matrix_t *temp3 = createMatrix_t(temp->row, temp2->col);
    Matrix_t_multiply(temp3, temp, temp2);
    // DEBUG_PRINT("KZKT = \n");
    // Matrix_t_print(temp3);
    Matrix_t_subtract(sigmaNew, sigmaP, temp3);
    Matrix_t_delete(temp);
    Matrix_t_delete(temp2);
    Matrix_t_delete(temp3);
    return true;
}

// 获取当前坐标
void getCurrentCoordinate(Coordinate *result)
{
    Matrix_t_copy(result->coordinateXYZ, currentCoordinate->coordinateXYZ);
}

void addMeasurementRecord(uint16_t sourceAdr,double d,uint16_t bufferId){
    xSemaphoreTake(measurementBufferMutex, portMAX_DELAY);
    if(bufferId < UKFBufferId){
        // 测距信息过期了
        DEBUG_PRINT("Measurement record is invaild\n");
        xSemaphoreGive(measurementBufferMutex);
        return;
    }
    // 查找positionId
    int positionId = -1;
    for (int i = 0; i < N_MEAS; i++)
    {
        if(edgerAdr[i] == sourceAdr){
            positionId = i;
            break;
        }
    }
    if(positionId == -1){
        DEBUG_PRINT("Position id is invaild\n");
        xSemaphoreGive(measurementBufferMutex);
        return;
    }
    // 添加测距信息
    measurementBuffer[positionId].coordinate->coordinateXYZ->data[0][0] = edgerCoordinate[positionId]->coordinateXYZ->data[0][0];
    measurementBuffer[positionId].coordinate->coordinateXYZ->data[1][0] = edgerCoordinate[positionId]->coordinateXYZ->data[1][0];
    measurementBuffer[positionId].coordinate->coordinateXYZ->data[2][0] = edgerCoordinate[positionId]->coordinateXYZ->data[2][0];
    measurementBuffer[positionId].d = d;
    hasNewMeasurement = true;
    xSemaphoreGive(measurementBufferMutex);
}

void UKFInit(){
    measurementBuffer = (MeasurementBufferNode_t*)malloc(sizeof(MeasurementBufferNode_t)*N_MEAS);
    edgerCoordinate = (Coordinate**)malloc(sizeof(Coordinate*)*N_MEAS);
    edgerAdr = (uint16_t*)malloc(sizeof(uint16_t)*N_MEAS);
    measurementBufferMutex = xSemaphoreCreateMutex();

    if(measurementBuffer == NULL || edgerCoordinate == NULL || measurementBufferMutex == NULL || edgerAdr == NULL){
        DEBUG_PRINT("UKF初始化失败\n");
        return;
    }

    for (int i = 0; i < N_MEAS; i++)
    {
        edgerCoordinate[i] = createCoordinate(0,0,0);
    }
    edgerAdr[0] = 0x01;
    setCoordinate(edgerCoordinate[0],0,0,0);
    edgerAdr[1] = 0x02;
    setCoordinate(edgerCoordinate[1],100,0,0);
    edgerAdr[2] = 0x03;
    setCoordinate(edgerCoordinate[2],0,100,0);
    edgerAdr[3] = 0x04;
    setCoordinate(edgerCoordinate[3],100,100,0);
    
    for (int i = 0; i < N_MEAS; i++)
    {
        measurementBuffer[i].coordinate = createCoordinate(0,0,0);
        measurementBuffer[i].d = 0;
    }
    DEBUG_PRINT("[UKFInit] 测量值缓冲区初始化成功\n");

    // 初始化UKF
    currentCoordinate = createCoordinate(0, 0, 0);
    currentVelocity = createVelocity(0, 0, 0);
    DEBUG_PRINT("[UKFInit] 当前坐标和速度初始化成功\n");

    currentMeasurementPoint = (Coordinate**)malloc(sizeof(Coordinate*)*N_MEAS);
    for (int i = 0; i < N_MEAS; i++)
    {
        currentMeasurementPoint[i] = createCoordinate(0,0,0);
    }
    zValue = createMeasurement(N_MEAS,2*N_STATE+1);
    zMean = createMeasurement(N_MEAS,1);
    zTrue = createMeasurement(N_MEAS,1);
    for(int i = 0;i<N_MEAS;++i){
        for(int j = 0;j<2*N_STATE+1;++j){
            zValue->measurement->data[i][j] = 0;
        }
    }
    for(int i = 0;i<N_MEAS;++i){
        zMean->measurement->data[i][0] = 0;
    }
    for(int i = 0;i<N_MEAS;++i){
        zTrue->measurement->data[i][0] = 0;
    }
    DEBUG_PRINT("[UKFInit] 测量值初始化成功\n");

    covX = createMatrix_t(N_STATE, N_STATE);
    covZ = createMatrix_t(N_MEAS, N_MEAS);
    covXZ = createMatrix_t(N_STATE, N_MEAS);
    K = createMatrix_t(N_STATE, N_MEAS);
    // 状态量方差矩阵初始化
    for(int i = 0;i<N_STATE;++i){
        Matrix_t_set(covX,i,i,10000);
    }
    DEBUG_PRINT("[UKFInit] 状态量方差矩阵初始化成功\n");

    // UT初始化
    sigmaPointInit();
    weightInit();
    noiseInit();
    DEBUG_PRINT("[UKFInit] UT初始化成功\n");

    DEBUG_PRINT("[UKFInit] UKF初始化成功\n");
}

void UKFTask(){
    systemWaitStart();

    vTaskDelay(M2T(1000));

    UKFInit();
    // 状态估计
    while (1)
    {
        xSemaphoreTake(measurementBufferMutex, portMAX_DELAY);
        if(hasNewMeasurement){
            // 存在测距信息，则进行更新
            // sigma点采样
            sigmaPointsUpdate(currentCoordinate, covX);

            // 量测值更新
            int top = 0;
            for (int i = 0; i < N_MEAS; i++){
                if(measurementBuffer[i].d == 0){
                    continue;
                }
                zTrue->measurement->data[top][0] = measurementBuffer[i].d;
                currentMeasurementPoint[top]->coordinateXYZ->data[0][0] = measurementBuffer[i].coordinate->coordinateXYZ->data[0][0];
                currentMeasurementPoint[top]->coordinateXYZ->data[1][0] = measurementBuffer[i].coordinate->coordinateXYZ->data[1][0];
                currentMeasurementPoint[top]->coordinateXYZ->data[2][0] = measurementBuffer[i].coordinate->coordinateXYZ->data[2][0];
                top++;
            }

            for (int i = 0; i < 2 * N_STATE + 1; i++)
            {
                for (int j = 0; j < top; j++)
                {
                    zValue->measurement->data[j][i] = equationsOfMeasurement(sigmaPoints[i],currentMeasurementPoint[j]);    
                }
                for(int j = top; j < N_MEAS; j++){
                    zTrue->measurement->data[j][0] = 0;
                    zValue->measurement->data[j][i] = 0;
                }
            }
            // 观测值均值和协方差更新
            meanOfMeasurements(zMean, zValue, weightM, 2 * N_STATE + 1);
            covarianceOfMeasurements(covZ, zValue, zMean, weightC, 2 * N_STATE + 1);

            // 计算状态量和观测量的协方差
            covarianceOfPointsAndMeasurements(covXZ, sigmaPoints, currentCoordinate, zValue, zMean, weightC, 2 * N_STATE + 1);

            // 更新卡尔曼滤波增益
            KUpdate(K, covXZ, covZ);

            // 状态量及协方差更新
            stateUpdate(currentCoordinate, currentCoordinate, K, zValue, zMean);
            covarianceUpdate(covX, covX, K, covZ);
        }
        // 获取当前速度
        getCurrentVelocity(currentVelocity);
        UKFBufferId++;
        // 清空测距信息
        for (int i = 0; i < N_MEAS; i++)
        {
            // d=0,即代表数据无效
            measurementBuffer[i].d = 0;
        }
        // 释放锁，及时更新MeasurementBuffer    
        xSemaphoreGive(measurementBufferMutex);

        // 基于当前坐标和速度进行动力学估计

        sigmaPointsUpdate(currentCoordinate, covX);
        // 动力学方程预测
        for (int i = 0; i < 2 * N_STATE + 1; i++)
        {
            equationsOfMotion(sigmaPoints[i], sigmaPoints[i], currentVelocity);
        }
        // 状态量均值和协方差更新
        meanOfPoints(currentCoordinate, sigmaPoints, weightM, 2 * N_STATE + 1);
        covarianceOfPoints(covX, sigmaPoints, currentCoordinate, weightC, 2 * N_STATE + 1);
        vTaskDelay(M2T(MOTION_ESTIMATE_INTERVAL));
    }
}

void UKFRelativePositionInit(){
    DEBUG_PRINT("[UKFRelativePositionInit]\n");

    xTaskCreate(UKFTask, UKF_TASK_NAME, UKF_TASK_STACKSIZE, NULL,
        UKF_TASK_PRI, &ukfTaskHandle);
    
}