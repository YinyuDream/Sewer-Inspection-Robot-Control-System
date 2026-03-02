/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h" // 引入 CAN 数据包定义
#include "power_management.h" // 引入电源管理业务逻辑层
#include "ekf.h" // 引入 EKF 算法接口
#include "control.h" // 引入运动控制算法接口
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Contorl */
osThreadId_t ContorlHandle;
const osThreadAttr_t Contorl_attributes = {
  .name = "Contorl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanDataCenter */
osThreadId_t CanDataCenterHandle;
const osThreadAttr_t CanDataCenter_attributes = {
  .name = "CanDataCenter",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PowerManagement */
osThreadId_t PowerManagementHandle;
const osThreadAttr_t PowerManagement_attributes = {
  .name = "PowerManagement",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EkfAlgorithm */
osThreadId_t EkfAlgorithmHandle;
const osThreadAttr_t EkfAlgorithm_attributes = {
  .name = "EkfAlgorithm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for powerManagementQueue */
osMessageQueueId_t powerManagementQueueHandle;
const osMessageQueueAttr_t powerManagementQueue_attributes = {
  .name = "powerManagementQueue"
};
/* Definitions for sensorEkfData */
osMessageQueueId_t sensorEkfDataHandle;
const osMessageQueueAttr_t sensorEkfData_attributes = {
  .name = "sensorEkfData"
};
/* Definitions for motionControl */
osMessageQueueId_t motionControlHandle;
const osMessageQueueAttr_t motionControl_attributes = {
  .name = "motionControl"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MotionControlTask(void *argument);
void CanCommunicationTask(void *argument);
void PowerHandleTask(void *argument);
void NavigationEkfTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (16, sizeof(CAN_RxPacketTypeDef), &canRxQueue_attributes);

  /* creation of powerManagementQueue */
  powerManagementQueueHandle = osMessageQueueNew (16, sizeof(Robot_general), &powerManagementQueue_attributes);

  /* creation of sensorEkfData */
  sensorEkfDataHandle = osMessageQueueNew (16, sizeof(Robot_general), &sensorEkfData_attributes);

  /* creation of motionControl */
  motionControlHandle = osMessageQueueNew (16, sizeof(Robot_general), &motionControl_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Contorl */
  ContorlHandle = osThreadNew(MotionControlTask, NULL, &Contorl_attributes);

  /* creation of CanDataCenter */
  CanDataCenterHandle = osThreadNew(CanCommunicationTask, NULL, &CanDataCenter_attributes);

  /* creation of PowerManagement */
  PowerManagementHandle = osThreadNew(PowerHandleTask, NULL, &PowerManagement_attributes);

  /* creation of EkfAlgorithm */
  EkfAlgorithmHandle = osThreadNew(NavigationEkfTask, NULL, &EkfAlgorithm_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MotionControlTask */
/**
  * @brief  Function implementing the Contorl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotionControlTask */
void MotionControlTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN MotionControlTask */
  float status[15]; // 15个位姿真值输入
  uint16_t PWM_Value[4]; // 有4个电机的PWM控制输入
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(motionControlHandle, &status, NULL, osWaitForever) == osOK)
    {
        // 这里可以处理运动控制相关的 CAN 数据，例如 位置、速度、加速度等
        // 将数据输入到运动控制算法中进行计算
        motion_control_algorithm(status, PWM_Value); // 运动控制算法处理，输出 PWM 控制值
        for (uint16_t i = 0; i < 4; i++) {
          CAN_Send_Data(0x180 + i, (uint8_t *)&PWM_Value[i], 2); // 将控制结果通过 CAN 发送出去，ID 从 0x180 开始
        }
    }
  }
  /* USER CODE END MotionControlTask */
}

/* USER CODE BEGIN Header_CanCommunicationTask */
/**
* @brief Function implementing the DataMerge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanCommunicationTask */
void CanCommunicationTask(void *argument)
{
  /* USER CODE BEGIN CanCommunicationTask */
  CAN_RxPacketTypeDef rxPacket;
  // uint8_t msg_buffer[64]; // 用于打印调试信息的缓冲区

  /* Infinite loop */
  for(;;)
  {
    // 1. 永久阻塞等待队列中有数据 (osWaitForever)
    if (osMessageQueueGet(canRxQueueHandle, &rxPacket, NULL, osWaitForever) == osOK)
    {
        // 收到 CAN 数据了！
        // 在这里进行业务逻辑处理，例如解析协议、控制电机等

        // 【示例逻辑】将收到的 CAN ID 和前 4 个字节数据通过 USB 打印出来调试
        // 注意：在多任务中并发调用 CDC_Transmit_FS 可能需要互斥锁，这里仅作演示
        /* int len = sprintf((char *)msg_buffer, "CAN Rx: ID=0x%lX, D0=%02X, D1=%02X\r\n", 
                          (unsigned long)rxPacket.header.StdId, rxPacket.data[0], rxPacket.data[1]);
        
        CDC_Transmit_FS(msg_buffer, len);*/
        
        Robot_general data;
        data.id = rxPacket.header.StdId;
        for(int i = 0; i < 4 && i < rxPacket.header.DLC; i++) {
            data.FloatBytes.bytes[i] = rxPacket.data[i];
        }

        if (rxPacket.header.StdId >= 0x000 && rxPacket.header.StdId <= 0x0FF) {
            // 电源管理相关数据和指令
            osMessageQueuePut(powerManagementQueueHandle, &data, 0, 0); // 可以将相关数据推入电源管理队列
        } else if (rxPacket.header.StdId >= 0x100 && rxPacket.header.StdId <= 0x1FF) {
            // 运动控制相关数据和指令
            osMessageQueuePut(motionControlHandle, &data, 0, 0); // 将相关数据推入运动控制队列
        } else if (rxPacket.header.StdId >= 0x200 && rxPacket.header.StdId <= 0x2FF) {
            // 传感器数据融合相关
            osMessageQueuePut(sensorEkfDataHandle, &data, 0, 0); // 将传感器数据推入 EKF 队列
        }else {
            // 其他 ID 的数据
        }
    }
  }
  /* USER CODE END CanCommunicationTask */
}

/* USER CODE BEGIN Header_PowerHandleTask */
/**
* @brief Function implementing the PowerManagement thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowerHandleTask */
void PowerHandleTask(void *argument)
{
  /* USER CODE BEGIN PowerHandleTask */
  /* Infinite loop */
  Robot_general data;
  float power_status[10]; // 假设有10个电源相关的状态数据
  float instruction[10]; // 假设有10个电源控制指令输入
  for(;;)
  {
    if (osMessageQueueGet(powerManagementQueueHandle, &data, NULL, osWaitForever) == osOK)
    {
        power_status[data.id] = data.FloatBytes.f;
        // 这里可以处理电源管理相关的 CAN 数据，例如监测电池电压、控制电源开关等
        power_management_logic(power_status, instruction); // 伪函数，代表电源管理的业务逻辑处理
        for (uint16_t i = 0; i < 10; i++) {
          CAN_Send_Data(0x080 + i, (uint8_t *)&power_status[i], 4);
        }
    }
  }
  /* USER CODE END PowerHandleTask */
}

/* USER CODE BEGIN Header_NavigationEkfTask */
/**
* @brief Function implementing the EkfAlgorithm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NavigationEkfTask */
void NavigationEkfTask(void *argument)
{
  /* USER CODE BEGIN NavigationEkfTask */
  /* Infinite loop */
  Robot_general data;
  float sensor_data[10]; // 假设有10个传感器数据输入到 EKF 中，例如 IMU 的加速度、角速度，里程计的速度等
  float result[10]; // 假设 EKF 输出也是 10 个状态变量
  for(;;)
  {
    if (osMessageQueueGet(sensorEkfDataHandle, &data, NULL, osWaitForever) == osOK)
    {
        // 这里可以处理传感器数据融合相关的 CAN 数据，例如 IMU、里程计等
        // 将数据输入到 EKF 算法中进行状态估计
        sensor_data[data.id - 0x200] = data.FloatBytes.f; // 假设前两个字节是一个传感器值，单位转换
        ekf_algorithm_update(sensor_data, result); // 伪函数，代表 EKF 算法的更新步骤
        for (uint16_t i = 0; i < 10; i++) {
          CAN_Send_Data(0x280 + i, (uint8_t *)&result[i], 4); // 将 EKF 结果通过 CAN 发送出去，ID 从 0x300 开始
        }
    }
  }
  /* USER CODE END NavigationEkfTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

