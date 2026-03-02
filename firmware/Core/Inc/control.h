#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#include <stdint.h>

/**
  * @brief 运动控制算法实现
  * @param status 机器人状态数据数组
  * @param PWM_Value 电机PWM控制值数组
  */
void motion_control_algorithm(float *status, uint16_t *PWM_Value);

#endif /* __MOTION_CONTROL_H */
