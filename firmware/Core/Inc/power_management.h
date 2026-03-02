#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

#include <stdint.h>

/**
  * @brief 电源管理业务逻辑核心处理接口
  * @param power_status 包含各个CAN节点电源状态的数据数组
  * @param instruction 包含各个CAN节点电源控制指令的数据数组
  */
void power_management_logic(float *power_status, float *instruction);

#endif /* __POWER_CONTROL_H */
