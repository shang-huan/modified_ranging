#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"

#include "UKF.h"
#include "stdlib.h"
#include "stdio.h"
#include "MatrixTool.h"
#include "UnscentedTransform.h"
#include "read_write_lock.h"

static TaskHandle_t ukfTaskHandle;
static Coordinate* currentCoordinate;

static Measurement* zValue;
static Measurement* zMean;
static Measurement* zTrue;

static Matrix_t *covX;
static Matrix_t *covZ;
static Matrix_t *covXZ;

static Matrix_t *K;

/*
UKFTask 所用数据
*/
uint16_t UKFBufferId;

static ReadWriteLock_t oldBufferReadWriteLock;
static uint16_t oldBufferStartSeq;
static uint16_t oldBufferTopIndex;
static UKFBufferNode_t* oldBuffer;

static ReadWriteLock_t newBufferReadWriteLock;
static uint16_t newBufferStartSeq;
static uint16_t newBufferTopIndex;
static UKFBufferNode_t* newBuffer;

static bool oldBufferIsModified = false;
static bool newBufferIsModified = false;

void UKFBufferNodeInit(UKFBufferNode_t* bufferNode){
    bufferNode->mutex = xSemaphoreCreateMutex();
    bufferNode->coordinate = createCoordinate(0,0,0);
    bufferNode->velocity = createVelocity(0,0,0);
    bufferNode->covX = createMatrix_t(3, 3);
    bufferNode->measurementBufferLen = 0;
    for (int i = 0; i < N_MEAS; i++)
    {
        bufferNode->measurementBuffer[i].coordinate = createCoordinate(0,0,0);
        bufferNode->measurementBuffer[i].d = 0;
    }
}

void UKFBufferArrayInit(UKFBufferNode_t* bufferArray, uint16_t length){
    for (int i = 0; i < length; i++)
    {
        UKFBufferNodeInit(&bufferArray[i]);
    }
}

bool addUKFBufferMeasurementRecord(double x,double y,double z,double d,uint16_t bufferId){
    if(bufferId < oldBufferStartSeq || bufferId >= newBufferStartSeq + newBufferTopIndex){
        DEBUG_PRINT("Measurement record is invaild\n");
        return false;
    }
    UKFBufferNode_t* target;
    if(bufferId >= newBufferStartSeq && bufferId - newBufferStartSeq < newBufferTopIndex){
        target = &newBuffer[bufferId - newBufferStartSeq];
    }else{
        target = &oldBuffer[bufferId - oldBufferStartSeq];
    }
    if(target->measurementBufferLen >= N_MEAS){
        DEBUG_PRINT("Measurement buffer is full\n");
        return false;
    }
    xSemaphoreTake(target->mutex, portMAX_DELAY);
    setCoordinate(target->measurementBuffer[target->measurementBufferLen].coordinate, x, y, z);
    target->measurementBufferLen++;
    xSemaphoreGive(target->mutex);
}

bool addUKFBufferMeasurementRecord_FixedPosition(int positionId,double d,uint16_t bufferId){
    static Coordinate** positions;
    if(positions == NULL){
        positions = (Coordinate**)malloc(sizeof(Coordinate*) * N_MEAS);
        for (int i = 0; i < N_MEAS; i++)
        {
            positions[i] = createCoordinate(0,0,0);
        }
        setCoordinate(positions[0],0,0,0);
        setCoordinate(positions[1],100,0,0);
        setCoordinate(positions[2],0,100,0);
        setCoordinate(positions[3],100,100,0);
    }
    if(positionId >= N_MEAS){
        DEBUG_PRINT("Position id is invaild\n");
        return false;
    }
    return addUKFBufferMeasurementRecord(positions[positionId]->coordinateXYZ->data[0][0],positions[positionId]->coordinateXYZ->data[1][0],positions[positionId]->coordinateXYZ->data[2][0],d,bufferId);
}
// 获取当前速度
void getCurrentVelocity(Velocity* velocity)
{
    // cm/s
    double vx = logGetFloat(logGetVarId("stateEstimate", "vx")) * 100; 
    double vy = logGetFloat(logGetVarId("stateEstimate", "vy")) * 100;
    double vz = logGetFloat(logGetVarId("stateEstimate", "vz")) * 100;
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

void UKFProcess(Coordinate* oldCoordinate,Velocity* oldVelocity,Matrix_t* oldCovX,UKFBufferNode_t* node,Coordinate* newCoordinate,Matrix_t* newCovX){
    if(node->measurementBufferLen == 0){
        // 动力学估计当前坐标
        equationsOfMotion(newCoordinate,oldCoordinate,oldVelocity);
        // 更新协方差
        Matrix_t_add(newCovX,oldCovX,Q);
    }else{
        // Sigma点采样
        sigmaPointsUpdate(oldCoordinate, oldCovX);
        // 动力学方程预测
        for (int j = 0; j < 2 * N_STATE + 1; j++)
        {
            equationsOfMotion(sigmaPoints[j], sigmaPoints[j], oldVelocity);
        }
        // 状态量均值和协方差更新
        meanOfPoints(oldCoordinate, sigmaPoints, weightM, 2 * N_STATE + 1);
        covarianceOfPoints(oldCovX, sigmaPoints, oldCoordinate, weightC, 2 * N_STATE + 1);
        // DEBUG_PRINT("currentCoordinate1 = \n");
        // Matrix_t_print(currentCoordinate->coordinateXYZ);

        sigmaPointsUpdate(oldCoordinate, oldCovX);
        //量测方程更新
        for (int j = 0; j < 2 * N_STATE + 1; j++)
        {
            for (int k = 0; k < node->measurementBufferLen; k++)
            {
                zValue->measurement->data[k][j] = equationsOfMeasurement(oldCoordinate,&node->measurementBuffer[k].coordinate);
                zTrue->measurement->data[k][0] = node->measurementBuffer->d;
            }
            for (int k = node->measurementBufferLen; k < N_MEAS; k++){
                zValue->measurement->data[k][j] = 0;
                zTrue->measurement->data[k][0] = 0;
            }
        }
        // 观测值均值和协方差更新
        meanOfMeasurements(zMean, zValue, weightM, 2 * N_STATE + 1);
        covarianceOfMeasurements(covZ, zValue, zMean, weightC, 2 * N_STATE + 1);
        
        // DEBUG_PRINT("zValue = \n");
        // Matrix_t_print(zValue->measurement);
        // DEBUG_PRINT("zMean = \n");
        // Matrix_t_print(zMean->measurement);
        // DEBUG_PRINT("covZ = \n");
        // Matrix_t_print(covZ);

        // 计算状态量和观测量的协方差
        covarianceOfPointsAndMeasurements(covXZ, sigmaPoints, oldCoordinate, zValue, zMean, weightC, 2 * N_STATE + 1);
        // DEBUG_PRINT("covXZ = \n");
        // Matrix_t_print(covXZ);

        // 更新卡尔曼滤波增益
        KUpdate(K, covXZ, covZ);
        // DEBUG_PRINT("K = \n");
        // Matrix_t_print(K);

        // 状态量及协方差更新
        stateUpdate(newCoordinate, oldCoordinate, K, zTrue, zMean);
        covarianceUpdate(newCovX, oldCovX, K, covZ);
    }
}

void UKFInit(){
    oldBuffer = (UKFBufferNode_t*)malloc(sizeof(UKFBufferNode_t)*MAX_UKF_BUFFER_LENGTH);
    newBuffer = (UKFBufferNode_t*)malloc(sizeof(UKFBufferNode_t)*MAX_UKF_BUFFER_LENGTH);
    UKFBufferArrayInit(oldBuffer,MAX_UKF_BUFFER_LENGTH);
    UKFBufferArrayInit(newBuffer,MAX_UKF_BUFFER_LENGTH);

    CreateReadWriteLock(&oldBufferReadWriteLock);
    CreateReadWriteLock(&newBufferReadWriteLock);

    oldBufferStartSeq = 0;
    oldBufferTopIndex = 0;

    newBufferStartSeq = MAX_UKF_BUFFER_LENGTH;
    newBufferTopIndex = 0;

    UKFBufferId = -1;

    // UT初始化
    sigmaPointInit();
    weightInit();
    noiseInit();
    // 当前坐标初始化
    if(currentCoordinate == NULL){
        currentCoordinate = createCoordinate(0, 0, 0);
    }
    // 状态量方差矩阵初始化
    if (covX == NULL)
    {
        covX = createMatrix_t(N_STATE, N_STATE);
        for(int i = 0;i<N_STATE;++i){
            Matrix_t_set(covX,i,i,5*5);
        }
    }
    // 观测值初始化
    if(zValue == NULL){
        zValue = createMeasurement(N_MEAS,2*N_STATE+1);
        zMean = createMeasurement(N_MEAS,1);
        zTrue = createMeasurement(N_MEAS,1);
        covZ = createMatrix_t(N_MEAS,N_MEAS);
    }
    // 协方差矩阵初始化
    if(covXZ == NULL){
        covXZ = createMatrix_t(N_STATE,N_MEAS);
    }
    // 卡尔曼增益初始化
    if (K == NULL)
    {
        K = createMatrix_t(N_STATE, N_MEAS);
    }
    // 初值缓存
    Matrix_t_copy(newBuffer[0].coordinate->coordinateXYZ ,currentCoordinate->coordinateXYZ);
    Matrix_t_copy(&newBuffer[0].covX,covX);
    newBufferTopIndex++;
}

void UKFTask(){
    UKFInit();
    // 状态估计
    while (1)
    {
        // 获取锁
        xSemaphoreTake(newBuffer[newBufferTopIndex].mutex, portMAX_DELAY);
        Velocity* lastVelocity;
        Coordinate* lastCoordinate;
        Matrix_t* lastCovX;
        if(newBufferTopIndex != 0){
            lastVelocity = newBuffer[newBufferTopIndex-1].velocity;
            lastCoordinate = newBuffer[newBufferTopIndex-1].coordinate;
            lastCovX = newBuffer[newBufferTopIndex-1].covX;
        }else{
            lastVelocity = oldBuffer[MAX_UKF_BUFFER_LENGTH-1].velocity;
            lastCoordinate = oldBuffer[MAX_UKF_BUFFER_LENGTH-1].coordinate;
            lastCovX = oldBuffer[MAX_UKF_BUFFER_LENGTH-1].covX;
        }
        Coordinate* currentCoordinate = newBuffer[newBufferTopIndex].coordinate;
        Matrix_t* currentCovX = newBuffer[newBufferTopIndex].covX;
        // 获取速度
        getCurrentVelocity(lastVelocity);
        // 动力学估计当前坐标
        equationsOfMotion(currentCoordinate,lastCoordinate,lastVelocity);
        // 更新协方差
        Matrix_t_add(currentCovX,lastCovX,Q);
        // 释放锁
        xSemaphoreGive(newBuffer[newBufferTopIndex].mutex);

        newBufferTopIndex++;
        UKFBufferId++;
        if(newBufferTopIndex == MAX_UKF_BUFFER_LENGTH){
            // 缓存满，批量处理量测数据
            // 旧缓存有新更新
            if(oldBufferIsModified){
                int i = 0;
                // 找到第一个具备观测信息数据
                for(i = 0;i<MAX_UKF_BUFFER_LENGTH;++i){
                    if(oldBuffer[i].measurementBufferLen != 0){
                        break;
                    }
                }
                for(;i<MAX_UKF_BUFFER_LENGTH;++i){
                    xSemaphoreTake(oldBuffer[i].mutex, portMAX_DELAY);
                    Velocity* historyVelocity = oldBuffer[i].velocity;
                    Coordinate* historyCoordinate = oldBuffer[i].coordinate;
                    Matrix_t* historyCovX = oldBuffer[i].covX;

                    Coordinate* nextCoodinate;
                    Matrix_t* nextCovX;
                    if(i == MAX_UKF_BUFFER_LENGTH-1){
                        xSemaphoreTake(newBuffer[0].mutex, portMAX_DELAY);
                        nextCoodinate = newBuffer[0].coordinate;
                        nextCovX = newBuffer[0].covX;
                    }else{
                        xSemaphoreTake(oldBuffer[i+1].mutex, portMAX_DELAY);
                        nextCoodinate = oldBuffer[i+1].coordinate;
                        nextCovX = oldBuffer[i+1].covX;    
                    }
                    // UKF
                    UKFProcess(historyCoordinate,historyVelocity,historyCovX,&(oldBuffer[i]),nextCoodinate,nextCovX);
                    // 释放锁
                    if(i == MAX_UKF_BUFFER_LENGTH-1){
                        xSemaphoreGive(newBuffer[0].mutex);
                    }else{
                        xSemaphoreGive(oldBuffer[i+1].mutex);
                    }
                    xSemaphoreGive(oldBuffer[i].mutex);
                }
            }
            // 处理新缓存区数据
            int i = 0;
            if(!oldBufferIsModified && !newBufferIsModified){
                for(i = 0;i<MAX_UKF_BUFFER_LENGTH;++i){
                    if(newBuffer[i].measurementBufferLen != 0){
                        break;
                    }
                }
            }
            for(;i<MAX_UKF_BUFFER_LENGTH-1;++i){
                xSemaphoreTake(newBuffer[i].mutex, portMAX_DELAY);
                Velocity* historyVelocity = newBuffer[i].velocity;
                Coordinate* historyCoordinate = newBuffer[i].coordinate;
                Matrix_t* historyCovX = newBuffer[i].covX;

                xSemaphoreTake(newBuffer[i+1].mutex, portMAX_DELAY);
                Coordinate* nextCoodinate = newBuffer[i+1].coordinate;
                Matrix_t* nextCovX = newBuffer[i+1].covX;

                // UKF处理
                UKFProcess(historyCoordinate,historyVelocity,historyCovX,&newBuffer[i],nextCoodinate,nextCovX);

                // 释放锁
                xSemaphoreGive(newBuffer[i+1].mutex);
                xSemaphoreGive(newBuffer[i].mutex);
            }
            // 新旧缓存区更替
            // 获取写锁
            TakeWriteLock(&newBufferReadWriteLock, portMAX_DELAY);
            TakeWriteLock(&oldBufferReadWriteLock, portMAX_DELAY);
            oldBufferStartSeq = newBufferStartSeq;
            newBufferStartSeq += MAX_UKF_BUFFER_LENGTH;
            UKFBufferNode_t* item = oldBuffer;
            oldBuffer = newBuffer;
            newBuffer = oldBuffer;
            // 逻辑清空
            newBufferTopIndex = 0;
            for(int j = 0;j<MAX_UKF_BUFFER_LENGTH;++j){
                oldBuffer[j].measurementBufferLen = 0;
                newBuffer[j].measurementBufferLen = 0;
            }
            // 修改修正标识
            oldBufferIsModified = false;
            newBufferIsModified = false;
            GiveWriteLock(&oldBufferReadWriteLock);
            GiveWriteLock(&newBufferReadWriteLock);
        }
        vTaskDelay(M2T(MOTION_ESTIMATE_INTERVAL));
    }
}

void UKFRelativePositionInit(){
    DEBUG_PRINT("[UKFRelativePositionInit]\n");
    xTaskCreate(UKFTask, UKF_TASK_NAME, UKF_TASK_STACKSIZE, NULL,
        UKF_TASK_PRI, &ukfTaskHandle);
    
}