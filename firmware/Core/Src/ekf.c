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
        // 伪操作：将传感器数据复制到结果数组中
        for (int i = 0; i < 10; i++) {
            result[i] = sensor_data[i];
        }
    }
}
