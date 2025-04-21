#ifndef UKFCONFIG_H
#define UKFCONFIG_H

// #define SIMULATION_MODEL

#define MAX_UKF_BUFFER_LENGTH 3

#define MOTION_ESTIMATE_INTERVAL 50

#define T (MOTION_ESTIMATE_INTERVAL / 1000.0) // 时间间隔

#define N_STATE 3 // 状态量维度
#define N_MEAS 4 // 测量量维度

#define BASE_VELOCITY 100

#define SIGMA_U 10 // 控制变量标准差
#define SIGMA_Z 10 // 观测值误差范围

/*
一般 alpha [1e-4,1],取较小值
beta 对于精确的高斯分布，则 beta=2是最优选择
kappa >= 0, 取3-n 或者 0
lambda = alpha^2 * (n + kappa) - n
*/
static double alpha = 0.01; //增大提高对新观测数据的敏感性，减小提高滤波稳定性
static double beta = 2.0;
static double kappa = 0; //调整分布的中心权重
static double lambda ;

#endif