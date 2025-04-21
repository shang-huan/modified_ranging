#include "FreeRTOS.h"
#include "debug.h"

#include "UnscentedTransform.h"
#include "MatrixTool.h"
#include "stdio.h"
#include "math.h"
#include "UKFconfig.h"

double* weightM; // 计算近似均值时权值
double* weightC; // 计算近似协方差时权值
Coordinate** sigmaPoints; // sigma点

/*
Q 过大 → 系统对状态变化过于敏感，导致估计结果剧烈波动（震荡）
Q 过小 → 系统对状态变化反应迟钝，难以跟随动态轨迹
*/
Matrix_t* Q; // 状态量噪声Q
/*
R 过大 → 系统对测量数据信任度低，估计结果主要依赖动力学模型。
R 过小 → 系统对测量数据过于依赖，容易受测距噪声影响，出现剧烈抖动。
*/
Matrix_t* R; // 量测量噪声R

// 初始化sigma点集
void sigmaPointInit()
{
    DEBUG_PRINT("alpha = %f\n",alpha);
    DEBUG_PRINT("beta = %f\n",beta);
    DEBUG_PRINT("kappa = %f\n",kappa);
    lambda = alpha * alpha * (N_STATE + kappa) - N_STATE;
    DEBUG_PRINT("lambda = %f\n",lambda);
    sigmaPoints = (Coordinate **)malloc(sizeof(Coordinate*) * (2 * N_STATE + 1));
    for (int i = 0; i < 2 * N_STATE + 1; i++)
    {
        sigmaPoints[i] = createCoordinate(0,0,0);
    }
}

// 初始化权重
void weightInit()
{
    weightM = (double *)malloc(sizeof(double) * (2 * N_STATE + 1));
    weightM[0] = lambda / (N_STATE + lambda);
    DEBUG_PRINT("weightM[0] = %f\n",weightM[0]);
    for (int i = 1; i < 2 * N_STATE + 1; i++)
    {
        weightM[i] = 1 / (2 * (N_STATE + lambda));
        DEBUG_PRINT("weightM[%d] = %f\n",i,weightM[i]);
    }

    weightC = (double *)malloc(sizeof(double) * (2 * N_STATE + 1));
    weightC[0] = lambda / (N_STATE + lambda) + (1 - alpha * alpha + beta);
    DEBUG_PRINT("weightC[0] = %f\n",weightC[0]);
    for (int i = 1; i < 2 * N_STATE + 1; i++)
    {
        weightC[i] = 1 / (2 * (N_STATE + lambda));
        DEBUG_PRINT("weightC[%d] = %f\n",i,weightC[i]);
    }
}

void noiseInit(){
    Q = createMatrix_t(N_STATE,N_STATE);
    R = createMatrix_t(N_MEAS,N_MEAS);

    setQ(SIGMA_U,SIGMA_U,SIGMA_U);

    for(int i = 0;i<N_MEAS;++i){
        Matrix_t_set(R,i,i,SIGMA_Z*SIGMA_Z);
    }
}

void setQ(double sigmaX, double sigmaY, double sigmaZ){
    if(Q == NULL){
        Q = createMatrix_t(N_STATE,N_STATE);
    }
    Matrix_t_set(Q,0,0,sigmaX*T*sigmaX*T);
    Matrix_t_set(Q,1,1,sigmaY*T*sigmaY*T);
    Matrix_t_set(Q,2,2,sigmaZ*T*sigmaZ*T);
}

//打印权重
void weightPrint(){
    for(int i = 0; i < 2 * N_STATE + 1; i++){
        DEBUG_PRINT("weightM[%d] = %f\n",i,weightM[i]);
    }
    for(int i = 0; i < 2 * N_STATE + 1; i++){
        DEBUG_PRINT("weightC[%d] = %f\n",i,weightC[i]);
    }
}

bool sigmaPointsUpdate(Coordinate *X, Matrix_t *P)
{
    // 更新sigma点
    static Matrix_t *L;
    if(L == NULL){
        L = createMatrix_t(P->row, P->col);
    }else{
        if(L->row != P->row || L->col != P->col){
            Matrix_t_delete(L);
            L = createMatrix_t(P->row, P->col);
        }
    }
    static Matrix_t *temp;
    if(temp == NULL){
        temp = createMatrix_t(P->row, 1);
    }else{
        if(temp->row != P->row){
            Matrix_t_delete(temp);
            temp = createMatrix_t(P->row, 1);
        }
    }
    Matrix_t_Cholesky(L, P);
    // Matrix_t_print(L);
    Matrix_t_copy(sigmaPoints[0]->coordinateXYZ, X->coordinateXYZ);
    for (int i = 1; i <= N_STATE; i++)
    {
        Matrix_t_get_col(temp, L, i);
        Matrix_t_scalar_multiply(temp, temp, sqrt(N_STATE + lambda));
        Matrix_t_add(sigmaPoints[i]->coordinateXYZ, X->coordinateXYZ, temp);
    }
    for (int i = N_STATE+1; i < 2 * N_STATE + 1; i++)
    {
        Matrix_t_get_col(temp, L, i - N_STATE);
        Matrix_t_scalar_multiply(temp, temp, sqrt(N_STATE + lambda));
        Matrix_t_subtract(sigmaPoints[i]->coordinateXYZ, X->coordinateXYZ, temp);
    }
}

bool meanOfPoints(Coordinate *result, Coordinate **points, double* weightM, int n){
    if(result == NULL || points == NULL){
        DEBUG_PRINT("result或points为空，请分配空间\n");
        return false;
    }
    for(int i=0;i<points[0]->coordinateXYZ->row;++i){
        result->coordinateXYZ->data[i][0] = 0;
        for (int j = 0; j < n; j++)
        {
            if(weightM == NULL){
                result->coordinateXYZ->data[i][0] += points[j]->coordinateXYZ->data[i][0]/n;   
            }else{
                result->coordinateXYZ->data[i][0] += points[j]->coordinateXYZ->data[i][0]*weightM[j];
            }
        }
    }
    return true;
}

bool  covarianceOfPoints(Matrix_t *result, Coordinate **points, Coordinate *mean, double* weightC, int n){
    if(result == NULL || points == NULL || mean == NULL){
        DEBUG_PRINT("result或points或mean为空，请分配空间\n");
        return false;
    }
    if(result->row != points[0]->coordinateXYZ->row || result->col != points[0]->coordinateXYZ->row){
        DEBUG_PRINT("result维度不匹配\n");
        return false;
    }
    static Matrix_t *centeredData;
    if(centeredData == NULL){
        centeredData = createMatrix_t(points[0]->coordinateXYZ->row,n);
    }else{
        if(centeredData->row != points[0]->coordinateXYZ->row || centeredData->col != n){
            Matrix_t_delete(centeredData);
            centeredData = createMatrix_t(points[0]->coordinateXYZ->row,n);
        }
    }
    static Matrix_t *centeredDataT;
    if(centeredDataT == NULL){
        centeredDataT = createMatrix_t(n,points[0]->coordinateXYZ->row);
    }else{
        if(centeredDataT->row != n || centeredDataT->col != points[0]->coordinateXYZ->row){
            Matrix_t_delete(centeredDataT);
            centeredDataT = createMatrix_t(points[0]->coordinateXYZ->row,n);
        }
    }
    for(int i = 0;i<points[0]->coordinateXYZ->row;++i){
        for(int j=0;j<n;++j){
            if(weightC == NULL){
                centeredData->data[i][j] = (points[j]->coordinateXYZ->data[i][0] - mean->coordinateXYZ->data[i][0])/n;
            }else{
                centeredData->data[i][j] = (points[j]->coordinateXYZ->data[i][0] - mean->coordinateXYZ->data[i][0])*weightC[j];
            }
            centeredDataT->data[j][i] = points[j]->coordinateXYZ->data[i][0] - mean->coordinateXYZ->data[i][0];
        }
    }
    Matrix_t_multiply(result,centeredData,centeredDataT);
    Matrix_t_add(result,result,Q);
}

// 计算量测均值
bool meanOfMeasurements(Measurement *mean, Measurement *measurements, double* weightM, int n){
    if(mean == NULL || measurements == NULL){
        DEBUG_PRINT("result或measurements为空，请分配空间\n");
        return false;
    }
    for (int i = 0; i < measurements->measurement->row; i++)
    {
        mean->measurement->data[i][0] = 0;
        for(int j = 0;j< measurements->measurement->col; ++j){
            if(weightM == NULL){
                mean->measurement->data[i][0] += (measurements->measurement->data[i][j] / measurements->measurement->col);
            }else{
                mean->measurement->data[i][0] += (measurements->measurement->data[i][j] * weightM[j]);
            }
        }        
    }
    return true;
}
// 计算量测协方差矩阵
bool covarianceOfMeasurements(Matrix_t *result, Measurement *measurements, Measurement *mean, double* weightC, int n){
    if(result == NULL || measurements == NULL || mean == NULL){
        DEBUG_PRINT("result或measurements或mean为空，请分配空间\n");
        return false;
    }
    if(result->row != measurements[0].measurement->row || result->col != measurements[0].measurement->row){
        DEBUG_PRINT("result维度不匹配\n");
        return false;
    }
    static Matrix_t *centeredData;
    if(centeredData == NULL){
        centeredData = createMatrix_t(measurements->measurement->row,measurements->measurement->col);
    }else{
        if(centeredData->row != measurements->measurement->row || centeredData->col != measurements->measurement->col){
            Matrix_t_delete(centeredData);
            centeredData = createMatrix_t(measurements->measurement->row,measurements->measurement->col);
        }
    }
    static Matrix_t *centeredDataT;
    if(centeredDataT == NULL){
        centeredDataT = createMatrix_t(measurements->measurement->col,measurements->measurement->row);
    }else{
        if(centeredDataT->row != measurements->measurement->col || centeredDataT->col != measurements->measurement->row){
            Matrix_t_delete(centeredDataT);
            centeredDataT = createMatrix_t(measurements->measurement->col,measurements->measurement->row);
        }
    }
    for(int i = 0;i<measurements->measurement->row;++i){
        for(int j=0;j<measurements->measurement->col;++j){
            if(weightC == NULL){
                centeredData->data[i][j] = (measurements->measurement->data[i][j] - mean->measurement->data[i][0])/n;
            }else{
                centeredData->data[i][j] = (measurements->measurement->data[i][j] - mean->measurement->data[i][0])*weightC[j];
            }
            centeredDataT->data[j][i] = measurements->measurement->data[i][j] - mean->measurement->data[i][0];
        }
    }
    Matrix_t_multiply(result,centeredData,centeredDataT);
    Matrix_t_add(result,result,R);
}

// 计算sigmaXZ
bool covarianceOfPointsAndMeasurements(Matrix_t *result, Coordinate **points, Coordinate *mean, Measurement *measurements, Measurement *meanZ, double* weightC, int n){
    if(result == NULL || points == NULL || mean == NULL || measurements == NULL || meanZ == NULL){
        DEBUG_PRINT("result或points或mean或measurements或meanZ为空，请分配空间\n");
        return false;
    }
    if(result->row != points[0]->coordinateXYZ->row || result->col != measurements[0].measurement->row){
        DEBUG_PRINT("result维度不匹配\n");
        return false;
    }
    static Matrix_t *centeredPointsData;
    if(centeredPointsData == NULL){
        centeredPointsData = createMatrix_t(points[0]->coordinateXYZ->row,n);
    }else{
        if(centeredPointsData->row != points[0]->coordinateXYZ->row || centeredPointsData->col != n){
            Matrix_t_delete(centeredPointsData);
            centeredPointsData = createMatrix_t(points[0]->coordinateXYZ->row,n);
        }
    }
    static Matrix_t *centeredMeasurementsDataT;
    if(centeredMeasurementsDataT == NULL){
        centeredMeasurementsDataT = createMatrix_t(measurements->measurement->col,measurements->measurement->row);
    }else{
        if(centeredMeasurementsDataT->row != measurements->measurement->col || centeredMeasurementsDataT->col != measurements->measurement->row){
            Matrix_t_delete(centeredMeasurementsDataT);
            centeredMeasurementsDataT = createMatrix_t(measurements->measurement->col,measurements->measurement->row);
        }
    }
    for (int i = 0; i < n; i++)
    {
        for(int j = 0; j<points[0]->coordinateXYZ->row;++j){
            centeredPointsData->data[j][i] = points[i]->coordinateXYZ->data[j][0] - mean->coordinateXYZ->data[j][0];
            if(weightC == NULL){
                centeredPointsData->data[j][i] = centeredPointsData->data[j][i] / n;
            }else{
                centeredPointsData->data[j][i] = centeredPointsData->data[j][i] * weightC[i];
            }
        }
        for(int j = 0; j<measurements->measurement->row;++j){
            centeredMeasurementsDataT->data[i][j] = measurements->measurement->data[j][i] - meanZ->measurement->data[j][0];
        }
    }
    Matrix_t_multiply(result,centeredPointsData,centeredMeasurementsDataT);
}