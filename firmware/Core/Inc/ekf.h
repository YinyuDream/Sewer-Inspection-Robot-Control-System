#ifndef __EKF_CONTROL_H
#define __EKF_CONTROL_H

#include <stdint.h>

/**
  * @brief EKF算法核心处理接口
  * @param sensor_data 传感器数据数组
  * @param result EKF算法输出结果数组
  */
void ekf_algorithm_update(float *sensor_data, float *result);

#endif /* __EKF_CONTROL_H */
