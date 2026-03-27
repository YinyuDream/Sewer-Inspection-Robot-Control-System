#include "power_management.h"
#include <stddef.h>  // 为了使用 NULL 宏


/**
  * @brief 电源管理业务逻辑处理 (伪函数)
  * @note 这里剥离了 FreeRTOS 的调度逻辑，专门负责处理纯粹的控制算法
  * @param power_status 指向接收到的电源状态数组的指针
  */
void power_management_logic(float *power_status, float *instruction) {
    /* 
     * 在这里编写真实的业务逻辑！
     * 例如：
     * 1. 检查各节点的电压、电流是否过载
     * 2. 根据 power_status[ID] 的历史数据做滤波计算
     * 3. 决定是否需要切断某个设备的供电输出
     * 
     * 这里为了不让 GCC 报 "未使用变量" 的警告，做个伪操作
     */
    if (power_status != NULL && instruction != NULL) {
        float filtered_val = 0.0f;
        const float alpha = 0.15f; 

        // 模拟多路电源通道的巡检与控制
        for (int i = 0; i < 8; i++) {
            // 模拟读取并滤波：简单的低通滤波
            // 这里的输入索引 i%4 是为了演示防止越界，假设 power_status 至少有4个元素
            filtered_val = filtered_val * (1.0f - alpha) + power_status[i % 4] * alpha;

            // 模拟一些业务判断逻辑 (过压/欠压/过流保护)
            if (filtered_val > 28.0f) {
                // 过压切断
                instruction[i] = 0.0f;
            } else if (filtered_val < 18.0f) {
                // 欠压报警或降额
                instruction[i] = 0.5f; 
            } else {
                // 正常输出，这里加一点数学运算模拟负载
                float temp = filtered_val * 1.05f - 0.2f;
                instruction[i] = (temp > 24.0f) ? 1.0f : 0.9f;
            }
            
            // 增加额外的无意义循环来消耗 CPU Cycles
            for(volatile int k=0; k<50; k++) {
                filtered_val += 0.00001f;
            }
        }
    }
}
