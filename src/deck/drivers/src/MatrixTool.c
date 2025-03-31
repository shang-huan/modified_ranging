#include "FreeRTOS.h"
#include "debug.h"

#include "MatrixTool.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

#define EPSILION 1e-8

// 创建矩阵
Matrix_t *createMatrix_t(int row, int col){
    Matrix_t *m = (Matrix_t *)malloc(sizeof(Matrix_t));
    m->row = row;
    m->col = col;
    m->data = (double **)malloc(sizeof(double *) * row);
    for(int i = 0; i < row; i++){
        m->data[i] = (double *)malloc(sizeof(double) * col);
        for(int j = 0; j < col; j++){
            m->data[i][j] = 0;
        }
    }
    return m;
} 
// 删除矩阵
void Matrix_t_delete(Matrix_t *m){
    for(int i = 0; i < m->row; i++){
        free(m->data[i]);
    }
    free(m->data);
    free(m);
}

// 打印矩阵
void Matrix_t_print(Matrix_t *m){
    DEBUG_PRINT("Matrix_t(%d,%d)\n", m->row, m->col);
    for(int i = 0; i < m->row; i++){
        for(int j = 0; j < m->col; j++){
            DEBUG_PRINT("%.4lf ", m->data[i][j]);
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

        DEBUG_PRINT("\n");
    }
}
// 设置矩阵元素
bool Matrix_t_set(Matrix_t *m, int row, int col, double val){
    if(m == NULL){
        DEBUG_PRINT("Matrix_t_set:m为空,请分配空间\n");
        return false;
    }
    if(m->row < row || m->col < col){
        DEBUG_PRINT("Matrix_t_set:设置矩阵元素错误,行或列超出范围\n");
        return false; 
    }
    m->data[row][col] = val;
    return true;
}
// 获取矩阵元素
double Matrix_t_get(Matrix_t *m, int row, int col){
    // todo: 异常处理
    return m->data[row][col];
}

 // 获取矩阵行元素
bool Matrix_t_get_row(Matrix_t* result, Matrix_t *m, int row){
    if(result == NULL || m == NULL){
        DEBUG_PRINT("Matrix_t_get_row:result为空,请分配空间\n");
        return false;
    }
    if(result->col != m->col || result->row != 1){
        DEBUG_PRINT("Matrix_t_get_row:获取行元素错误,result维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m->col; i++){
        result->data[0][i] = m->data[row][i];
    }
    return true;
}
// 设置行元素
bool Matrix_t_set_row(Matrix_t *m, Matrix_t *val, int row){
    // 判空
    if(m == NULL || val == NULL){
        DEBUG_PRINT("Matrix_t_set_row:m或val为空,请分配空间\n");
        return false;
    }
    if(m->col != val->col){
        DEBUG_PRINT("Matrix_t_set_row:设置行元素错误,列长度不匹配\n");
        return false;
    }
    if(m->row < row){
        DEBUG_PRINT("Matrix_t_set_row:设置行元素错误,行数超出范围\n");
        return false;
    }
    for(int i = 0; i < m->col; i++){
        m->data[row][i] = val->data[0][i];
    }
    return true;
}
 // 获取矩阵列元素
bool Matrix_t_get_col(Matrix_t* result, Matrix_t *m, int col){
    if(result == NULL || m == NULL){
        DEBUG_PRINT("Matrix_t_get_col:result或m为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != 1){
        DEBUG_PRINT("Matrix_t_get_col:获取列元素错误,result维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m->row; i++){
        result->data[i][0] = m->data[i][col];
    }
    return true;
}
// 设置列元素
bool Matrix_t_set_col(Matrix_t *m, Matrix_t *val, int col){
    if(m == NULL || val == NULL){
        DEBUG_PRINT("Matrix_t_set_col:m或val为空,请分配空间\n");
        return false;
    }
    if(m->row != val->row){
        DEBUG_PRINT("Matrix_t_set_col:设置列元素错误,行长度不匹配\n");
        return false;
    }
    if(m->col < col){
        DEBUG_PRINT("Matrix_t_set_col:设置列元素错误,列数超出范围\n");
        return false;
    }
    for(int i = 0; i < m->row; i++){
        m->data[i][col] = val->data[i][0];
    }
    return true;
}
// 复制矩阵
bool Matrix_t_copy(Matrix_t* result, Matrix_t *m){
    if(result == NULL || m == NULL){
        DEBUG_PRINT("Matrix_t_copy:result或m为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != m->col){
        DEBUG_PRINT("Matrix_t_copy:复制矩阵错误,result维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m->row; i++){
        for(int j = 0; j < m->col; j++){
            result->data[i][j] = m->data[i][j];
        }
    }
    return true;
}
// 矩阵加法
bool Matrix_t_add(Matrix_t* result, Matrix_t *m1, Matrix_t *m2){
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_add:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m1->row || result->col != m1->col){
        DEBUG_PRINT("Matrix_t_add:矩阵加法错误,result维度不匹配\n");
        return false;
    }
    if(m1->row != m2->row || m1->col != m2->col){
        DEBUG_PRINT("Matrix_t_add:矩阵加法错误,矩阵维度不匹配,A(%d,%d),B(%d,%d)\n",m1->row,m1->col,m2->row,m2->col);
        return false;
    }
    for(int i = 0; i < m1->row; i++){
        for(int j = 0; j < m1->col; j++){
            result->data[i][j] = m1->data[i][j] + m2->data[i][j];
            // DEBUG_PRINT("result->data[%d][%d] = %lf\n",i,j,result->data[i][j]);
        }
    }
    return true;
}
// 矩阵减法
bool Matrix_t_subtract(Matrix_t* result, Matrix_t *m1, Matrix_t *m2){
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_subtract:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m1->row || result->col != m1->col){
        DEBUG_PRINT("Matrix_t_subtract:矩阵减法错误,result维度不匹配\n");
        return false;
    }
    if(m1->row != m2->row || m1->col != m2->col){
        DEBUG_PRINT("Matrix_t_subtract:矩阵减法错误,矩阵维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m1->row; i++){
        for(int j = 0; j < m1->col; j++){
            result->data[i][j] = m1->data[i][j] - m2->data[i][j];
        }
    }
    return true;
}
// 矩阵乘法
bool Matrix_t_multiply(Matrix_t* result, Matrix_t *m1, Matrix_t *m2){
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_multiply:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m1->row || result->col != m2->col){
        DEBUG_PRINT("Matrix_t_multiply:矩阵乘法错误,result维度不匹配\n");
        return false;
    }
    if(m1->col != m2->row){
        DEBUG_PRINT("Matrix_t_multiply:矩阵乘法错误,矩阵维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m1->row; i++){
        for(int j = 0; j < m2->col; j++){
            result->data[i][j] = 0;
            for(int k = 0; k < m1->col; k++){
                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }
    return true;
}
// 矩阵数乘
bool Matrix_t_scalar_multiply(Matrix_t* result, Matrix_t *m, double scalar){
    if(result == NULL){
        // DEBUG_PRINT("矩阵result为空,分配空间\n");
        DEBUG_PRINT("Matrix_t_scalar_multiply:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != m->col){
        DEBUG_PRINT("Matrix_t_scalar_multiply:矩阵数乘错误,result维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m->row; i++){
        for(int j = 0; j < m->col; j++){
            result->data[i][j] = (m->data[i][j] * scalar);
        }
    }
    return true;
}
// 矩阵乘幂
bool Matrix_t_pow(Matrix_t* result, Matrix_t *m, int power){
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_pow:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != m->col){
        DEBUG_PRINT("Matrix_t_pow:矩阵乘方错误,result维度不匹配\n");
        return false;
    }
    if(m->row != m->col){
        DEBUG_PRINT("Matrix_t_pow:矩阵乘方错误,矩阵不是方阵\n");
        return false;
    }
    Matrix_t *temp = createMatrix_t(m->row, m->col);
    Matrix_t_copy(temp,m);
    Matrix_t *temp2 = createMatrix_t(m->row, m->col);
    for(int i = 0; i < m->row; i++){
        result->data[i][i] = 1;
    }
    // todo
    Matrix_t * t = NULL;
    while(power){
        if(power & 1){
            Matrix_t_multiply(temp2, result, temp);
            t = result;
            result = temp2;
            temp2 = t;
        }
        Matrix_t_multiply(temp2, temp, temp);
        t = temp;
        temp = temp2;
        temp2 = t;
        power >>= 1;
    }
    Matrix_t_delete(temp);
    Matrix_t_delete(temp2);
    return true;
}
// 矩阵转置
bool Matrix_t_transpose(Matrix_t* result, Matrix_t *m){
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_transpose:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m->col || result->col != m->row){
        DEBUG_PRINT("Matrix_t_transpose:矩阵转置错误,result维度不匹配\n");
        return false;
    }
    for(int i = 0; i < m->row; i++){
        for(int j = 0; j < m->col; j++){
            result->data[j][i] = m->data[i][j];
        }
    }
    return true;
}
// 矩阵求逆
// todo 核查逻辑
bool Matrix_t_inverse(Matrix_t* result, Matrix_t *m){
    if(m->row != m->col){
        DEBUG_PRINT("Matrix_t_inverse:矩阵求逆错误,矩阵不是方阵\n");
        return false;
    }
    if(result == NULL){
        DEBUG_PRINT("Matrix_t_inverse:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != m->col){
        DEBUG_PRINT("Matrix_t_inverse:矩阵求逆错误,result维度不匹配\n");
        return false;
    }
    if(Matrix_t_determinant(m)==0){
        DEBUG_PRINT("Matrix_t_inverse:矩阵行列式为0,不可求逆\n");
        return false;
    }
    Matrix_t *temp = createMatrix_t(m->row, m->col);
    Matrix_t_copy(temp,m);
    for(int i = 0; i < m->row; i++){
        result->data[i][i] = 1;
    }
    for(int i = 0; i < m->row; i++){
        if(temp->data[i][i] == 0){
            for(int j = i + 1; j < m->row; j++){
                if(temp->data[j][i] != 0){
                    for(int k = 0; k < m->row; k++){
                        temp->data[i][k] += temp->data[j][k];
                        result->data[i][k] += result->data[j][k];
                    }
                    break;
                }
            }
        }
        double t = temp->data[i][i];
        for(int j = 0; j < m->row; j++){
            temp->data[i][j] /= t;
            result->data[i][j] /= t;
        }
        for(int j = 0; j < m->row; j++){
            if(j == i){
                continue;
            }
            t = temp->data[j][i];
            for(int k = 0; k < m->row; k++){
                temp->data[j][k] -= temp->data[i][k] * t;
                result->data[j][k] -= result->data[i][k] * t;
            }
        }
    }
    Matrix_t_delete(temp);
    return true;
}
// 矩阵求行列式
double Matrix_t_determinant(Matrix_t *m){
    if(m->row != m->col){
        DEBUG_PRINT("Matrix_t_determinant:矩阵求行列式错误,矩阵不是方阵\n");
        return -1;
    }
    Matrix_t *temp = createMatrix_t(m->row, m->col);
    Matrix_t_copy(temp,m);
    double result = 1;
    for(int i = 0; i < m->row; i++){
        if(temp->data[i][i] == 0){
            for(int j = i + 1; j < m->row; j++){
                if(temp->data[j][i] != 0){
                    for(int k = 0; k < m->row; k++){
                        temp->data[i][k] += temp->data[j][k];
                    }
                    result *= -1;
                    break;
                }
            }
        }
        double t = temp->data[i][i];
        for(int j = 0; j < m->row; j++){
            temp->data[i][j] /= t;
        }
        for(int j = 0; j < m->row; j++){
            if(j == i){
                continue;
            }
            t = temp->data[j][i];
            for(int k = 0; k < m->row; k++){
                temp->data[j][k] -= temp->data[i][k] * t;
            }
        }
    }
    for(int i = 0; i < m->row; i++){
        result *= temp->data[i][i];
    }
    Matrix_t_delete(temp);
    return result;
}
// 矩阵Cholesky分解
bool Matrix_t_Cholesky(Matrix_t* result, Matrix_t *m){
    if(result == NULL)
    {
        DEBUG_PRINT("Matrix_t_Cholesky:result为空,请分配空间\n");
        return false;
    }
    if(result->row != m->row || result->col != m->col){
        DEBUG_PRINT("Matrix_t_Cholesky:矩阵Cholesky分解错误,result维度不匹配\n");
        return false;
    }
    if(m->row != m->col){
        DEBUG_PRINT("Matrix_t_Cholesky:矩阵Cholesky分解错误,矩阵不是方阵\n");
        return false;
    }
    if(isZeroMatrix(m)){
        DEBUG_PRINT("[Matrix_t_Cholesky]原始矩阵为0矩阵\n");
        return true;
    }
    for(int i = 0; i < m->row; i++){
        for(int j = 0; j <= i; j++){
            if(j==i){
                double sum = 0;
                for(int k = 0; k < j; k++){
                    sum += result->data[j][k] * result->data[j][k];
                }
                if(m->data[j][j] - sum <= EPSILION){
                    DEBUG_PRINT("Matrix_t_Cholesky:矩阵Cholesky分解错误,矩阵不是正定矩阵\n");
                    return false;
                }
                result->data[j][j] = sqrt(m->data[j][j] - sum);
            }
            else{
                double sum = 0;
                for(int k = 0; k < j; k++){
                    sum += result->data[i][k] * result->data[j][k];
                }                
                result->data[i][j] = 1.0 / result->data[j][j] * (m->data[i][j] - sum);
            }
        }
    }
    return true;
}

bool calCov(Matrix_t* res,Matrix_t* A,Matrix_t* B){
    if(A->row != B->row){
        DEBUG_PRINT("维度不匹配\n");
    }
    Matrix_t* meanA = createMatrix_t(A->row,1);
    Matrix_t* meanB = createMatrix_t(B->row,1);
    for (int i = 0; i < A->row; i++)
    {
        for (int j = 0; j < A->col; j++)
        {
            meanA->data[i][0] += A->data[i][j];
        }
        for (int j = 0; j < B->col; j++)
        {
            meanB->data[i][0] += B->data[i][j];
        }
    }
    for (int i = 0; i < A->row; i++)
    {
        meanA->data[i][0] = meanA->data[i][0]/(A->col * 1.0);
        meanB->data[i][0] = meanB->data[i][0]/(B->col * 1.0);
    }
    Matrix_t* itemA = createMatrix_t(A->row,A->col);
    Matrix_t* itemBT = createMatrix_t(B->row,B->col);
    for (int i = 0; i < A->row; i++)
    {
        for (int j = 0; j < A->col; j++)
        {
            itemA->data[i][j] = A->data[i][j] - meanA->data[i][0];
        }
        for (int j = 0; j < B->col; j++)
        {
            itemBT->data[j][i] = B->data[i][j] - meanB->data[i][0];
        }
    }
    Matrix_t_multiply(res,itemA,itemBT);
    return true;
}

bool isZeroMatrix(Matrix_t *m){
    for (int i = 0; i < m->row; i++)
    {
        for (int j = 0; j < m->col; j++)
        {
            if(m->data[i][j] != 0){
                return false;
            }
        }
    }
    return true;
}