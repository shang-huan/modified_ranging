#ifndef Matrix_tTOOL_H
#define Matrix_tTOOL_H
#include "stdbool.h"

typedef struct matrix
{
    int row;
    int col;
    double **data;
} Matrix_t;

Matrix_t *createMatrix_t(int row, int col);                    // 创建矩阵
void Matrix_t_delete(Matrix_t *m);                             // 删除矩阵
void Matrix_t_print(Matrix_t *m);                              // 打印矩阵
bool Matrix_t_set(Matrix_t *m, int row, int col, double val);  // 设置矩阵元素
double Matrix_t_get(Matrix_t *m, int row, int col);            // 获取矩阵元素
bool Matrix_t_get_row(Matrix_t *result, Matrix_t *m, int row); // 获取矩阵行元素
bool Matrix_t_set_row(Matrix_t *m, Matrix_t *val, int row);    // 设置行元素
bool Matrix_t_get_col(Matrix_t *result, Matrix_t *m, int col); // 获取矩阵列元素
bool Matrix_t_set_col(Matrix_t *m, Matrix_t *val, int col);    // 设置列元素
bool Matrix_t_copy(Matrix_t *result, Matrix_t *m);             // 复制矩阵

bool isZeroMatrix(Matrix_t *m);

bool Matrix_t_add(Matrix_t *result, Matrix_t *m1, Matrix_t *m2);             // 矩阵加法
bool Matrix_t_subtract(Matrix_t *result, Matrix_t *m1, Matrix_t *m2);        // 矩阵减法
bool Matrix_t_multiply(Matrix_t *result, Matrix_t *m1, Matrix_t *m2);        // 矩阵乘法
bool Matrix_t_scalar_multiply(Matrix_t *result, Matrix_t *m, double scalar); // 矩阵数乘
bool Matrix_t_pow(Matrix_t *result, Matrix_t *m, int power);                 // 矩阵幂
bool Matrix_t_transpose(Matrix_t *result, Matrix_t *m);                      // 矩阵转置
bool Matrix_t_inverse(Matrix_t *result, Matrix_t *m);                        // 矩阵求逆
double Matrix_t_determinant(Matrix_t *m);                                    // 矩阵求行列式

bool calCov(Matrix_t* res,Matrix_t* A,Matrix_t* B);//计算向量组协方差

bool Matrix_t_Cholesky(Matrix_t *result, Matrix_t *m); // Cholesky分解
#endif
