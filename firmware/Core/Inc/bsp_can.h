#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "can.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

// 1. 定义一个打包 CAN 接收数据的结构体，用于 FreeRTOS 队列传递
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_RxPacketTypeDef;

typedef struct {
    union {
        float f[2];
        uint8_t bytes[8];
    } FloatBytes;
    uint32_t id;
} Robot_general;

// 2. 声明外部的 FreeRTOS 消息队列句柄 (之后我们将在 freertos.c 中实际定义它)
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t powerManagementQueueHandle;
extern osMessageQueueId_t sensorEkfDataHandle;

/* CAN transmit queue (created in freertos.c) */
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxPacketTypeDef;

extern osMessageQueueId_t canTxQueueHandle;

// 函数声明
void CAN_Filter_Config(void);
void CAN_Send_Data(uint16_t id, uint8_t *data, uint8_t len);

#endif /* __BSP_CAN_H */
