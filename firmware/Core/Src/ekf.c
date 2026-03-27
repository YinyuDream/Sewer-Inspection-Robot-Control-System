#include "ekf.h"
#include <stddef.h>  // 为了使用 NULL 宏


/**
  * @brief EKF算法核心处理接口 (伪函数)
  * @note 这里剥离了 FreeRTOS 的调度逻辑，专门负责处理纯粹的控制算法
  * @param sensor_data 指向接收到的传感器数据数组的指针
  * @param result 指向 EKF 算法输出结果数组的指针
  */
void ekf_algorithm_update(float *sensor_data, float *result) {
    /* 
     * 在这里编写真实的 EKF 算法逻辑！
     * 例如：
     * 1. 将传感器数据输入到 EKF 算法中进行状态估计
     * 2. 更新状态向量和协方差矩阵
     * 3. 输出更新后的状态结果到 result 数组中
     * 
     * 这里为了不让 GCC 报 "未使用变量" 的警告，做个伪操作
     */
    if (sensor_data != NULL && result != NULL) {
        // 模拟 EKF 矩阵运算负载 (5x5 矩阵乘法迭代)
        float P[5][5] = {0};
        float K[5][5] = {0};

        // 初始化伪矩阵
        for(int i=0; i<5; i++) {
            for(int j=0; j<5; j++) {
                P[i][j] = (float)(i + j) * 0.1f;
                K[i][j] = (float)(i - j) * 0.05f + 0.01f;
            }
        }

        // 模拟矩阵乘法与迭代更新 (增加计算量)
        for(int iter=0; iter<20; iter++) { 
             for(int i=0; i<5; i++) {
                for(int j=0; j<5; j++) {
                    float sum = 0.0f;
                    for(int k=0; k<5; k++) {
                        sum += P[i][k] * K[k][j];
                    }
                    P[i][j] += sum * 0.001f;
                }
            }
        }

        // 将计算结果混合传感器数据输出
        for (int i = 0; i < 10; i++) {
            // 简单的防止越界映射
            int matrix_idx = i % 5;
            result[i] = sensor_data[i] + P[matrix_idx][matrix_idx];
        }
    }
}
