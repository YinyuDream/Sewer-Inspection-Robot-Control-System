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
#include "FreeRTOSConfig.h" // 用于 SystemCoreClock 和 portGET_RUN_TIME_COUNTER_VALUE
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
/* CAN transmit queue */
osMessageQueueId_t canTxQueueHandle;
const osMessageQueueAttr_t canTxQueue_attributes = {
  .name = "canTxQueue"
};



/* CAN Tx task handle */
osThreadId_t CanTxHandle;
const osThreadAttr_t CanTx_attributes = {
  .name = "CanTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PowerManagement */
osThreadId_t PowerManagementHandle;
const osThreadAttr_t PowerManagement_attributes = {
  .name = "PowerManagement",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for EkfAlgorithm */
osThreadId_t EkfAlgorithmHandle;
const osThreadAttr_t EkfAlgorithm_attributes = {
  .name = "EkfAlgorithm",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
void MonitorTask(void *argument);
void CAN_TxTask(void *argument);
static inline uint64_t get_hardware_time_us(void) {
    /* Use DWT cycle counter for microsecond resolution */
    uint32_t cycles = portGET_RUN_TIME_COUNTER_VALUE(); /* 32-bit counter, overflows every ~25.5s at 168MHz */
    /* Convert cycles to microseconds: cycles * 1,000,000 / SystemCoreClock */
    return (uint64_t)cycles * 1000000ULL / SystemCoreClock;
}
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
  MX_USB_DEVICE_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* mutexes removed; using queue for CAN Tx */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (64, sizeof(CAN_RxPacketTypeDef), &canRxQueue_attributes);

  /* creation of powerManagementQueue */
  powerManagementQueueHandle = osMessageQueueNew (32, sizeof(Robot_general), &powerManagementQueue_attributes);

  /* creation of sensorEkfData */
  sensorEkfDataHandle = osMessageQueueNew (32, sizeof(Robot_general), &sensorEkfData_attributes);

  /* creation of motionControl */
  motionControlHandle = osMessageQueueNew (32, sizeof(Robot_general), &motionControl_attributes);

  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* creation of canTxQueue */
  canTxQueueHandle = osMessageQueueNew (32, sizeof(CAN_TxPacketTypeDef), &canTxQueue_attributes);

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
  /* creation of CAN Tx task */
  CanTxHandle = osThreadNew(CAN_TxTask, NULL, &CanTx_attributes);

  /* creation of MonitorTask */
  const osThreadAttr_t MonitorTask_attributes = {
    .name = "MonitorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityLow,
  };
  osThreadNew(MonitorTask, NULL, &MonitorTask_attributes);
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
  /* USER CODE BEGIN MotionControlTask */
  Robot_general data;
  float status[20]; // 20个位姿真值输入
  float status_now[20];
  uint16_t PWM_Value[4]; // 有4个电机的PWM控制输入
  const uint32_t EXEC_PERIOD_US = 20 * 1000; // 执行周期 10ms => 10000us
  uint64_t exec_tick_us = 0; // 算法执行耗时（us）

  memset(status, 0, sizeof(status));

  /* Infinite loop */
  for(;;)
  {
    // 等待控制触发标志（由 CAN 接收线程在收到 0x300 时设置）
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    uint32_t cnt = 0;
    // 清空队列更新状态
    while (osMessageQueueGet(motionControlHandle, &data, NULL, 0) == osOK) {
        if (data.id >= 0x100 && data.id <= 0x107) {
            cnt++;
            int base_idx = (data.id - 0x100) * 2;
            if (base_idx < 20) {
                status[base_idx] = data.FloatBytes.f[0];
            }
            if (base_idx + 1 < 20) {
                status[base_idx + 1] = data.FloatBytes.f[1];
            }
        }
        else if (data.id == 0x300) {
            cnt++;
            // 0x300 包含 dt 和 0.0
            status[16] = data.FloatBytes.f[0]; // dt
            status[17] = data.FloatBytes.f[1]; // reserved
        }
    }
    memcpy(status_now, status, sizeof(status)); // 备份当前状态，供算法使用
    uint64_t start_tick_us = get_hardware_time_us();

    // 执行运动控制算法
    motion_control_algorithm(status_now, PWM_Value); // 运动控制算法处理，输出 PWM 控制值

    // 发送控制结果：ID 0x180 包含4个uint16 (8字节)
    CAN_Send_Data(0x180, (uint8_t *)PWM_Value, 8);

    // 计算算法单次执行耗时（ms）并发送
    exec_tick_us = get_hardware_time_us() - start_tick_us;
    data.FloatBytes.f[0] = (float)exec_tick_us / 1000.0f;  // 转换为毫秒
    CAN_Send_Data(0x400, data.FloatBytes.bytes, 4);
    CAN_Send_Data(0x301,(uint8_t*)&cnt,4);
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
  Robot_general data;
  /* Infinite loop */
  for(;;)
  {
    // 1. 永久阻塞等待队列中有数据 (osWaitForever)
    if (osMessageQueueGet(canRxQueueHandle, &rxPacket, NULL, osWaitForever) == osOK)
    {
        // 收到 CAN 数据了！
        // 在这里进行业务逻辑处理，例如解析协议、控制电机等
        
        data.id = rxPacket.header.StdId;
        // Copy up to 8 bytes (two floats)
        for(int i = 0; i < 8 && i < rxPacket.header.DLC; i++) {
          data.FloatBytes.bytes[i] = rxPacket.data[i];
        }

        /* 如果收到仿真时钟包 (CAN ID 0x300, DLC>=4)，解析为两个float：dt和0.0 */
        if (rxPacket.header.StdId == 0x300 && rxPacket.header.DLC >= 8) {
          // 解析为两个float：dt（秒）和0.0
          float dt_seconds;
          float dummy;
          memcpy(&dt_seconds, &rxPacket.data[0], 4);
          memcpy(&dummy, &rxPacket.data[4], 4);
          osMessageQueuePut(motionControlHandle, &data, 0, 0); // 将 dt 包含在 motion control 的队列中
          // 触发运动控制任务和EKF任务
          osThreadFlagsSet(ContorlHandle, 0x01);
          osThreadFlagsSet(EkfAlgorithmHandle, 0x02);
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
        } else if (rxPacket.header.StdId == 0x700) {
            // 环回测试接口 (Ping-Pong Test)，用于测试丢包率和总线延迟
            CAN_Send_Data(0x701, rxPacket.data, rxPacket.header.DLC);
        } else {
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
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz = 100ms周期
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
    // 等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // 非阻塞方式检查队列，更新数据
    while (osMessageQueueGet(powerManagementQueueHandle, &data, NULL, 0) == osOK)
    {
        // 接收到数据立即更新（注意：这里假设 data.id 在 0-9 范围内）
        if (data.id < 10) {
            power_status[data.id - 0x000] = data.FloatBytes.f[0];
        }
    }

    // 执行电源管理业务逻辑
    power_management_logic(power_status, instruction);

    // 发送 CAN 数据 (示例：注释掉实际发送)
    for (uint16_t i = 0; i < 10; i++) {
      // CAN_Send_Data(0x080 + i, (uint8_t *)&power_status[i], 4);
      // osDelay(1);
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
    // 等待 EKF 触发标志（由 CAN 接收线程在收到 0x300 时设置）
    osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
    
    // 清空队列更新传感器数据
    while (osMessageQueueGet(sensorEkfDataHandle, &data, NULL, 0) == osOK)
    {
        // 接收到数据立即更新（注意：这里假设 data.id 在 0x200-0x2FF 范围内）
        if (data.id >= 0x200 && data.id <= 0x2FF) {
            // 每个 CAN ID 包含两个 float，存储到连续位置
            int base_idx = (data.id - 0x200) * 2;
            if (base_idx < 10) {
                sensor_data[base_idx] = data.FloatBytes.f[0];
            }
            if (base_idx + 1 < 10) {
                sensor_data[base_idx + 1] = data.FloatBytes.f[1];
            }
        }
    }

    // 执行 EKF 算法更新
    ekf_algorithm_update(sensor_data, result); // 伪函数，代表 EKF 算法的更新步骤
    
    // 发送 EKF 结果
    for (uint16_t i = 0; i < 10; i++) {
      // CAN_Send_Data(0x280 + i, (uint8_t *)&result[i], 4); // 将 EKF 结果通过 CAN 发送出去，ID 从 0x300 开始
      // osDelay(1);
    }
  }
  /* USER CODE END NavigationEkfTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MonitorTask(void *argument)
{
  /* 
   * 获取各个任务堆栈的高水位线（即史上最小剩余可用空间）
   * 如果某个任务高水位线经常返回很小的值（如10以下），则说明有栈溢出的风险
   */
  Robot_general data;
  data.id = 0x500; // 专属监控基准ID，后续+1区分不同任务

  for(;;)
  {
    /* 每2秒发送一次堆栈水位监控数据 */
    osDelay(2000);

    // 1. 发送 MotionControlTask (ContorlHandle) 的高水位 [ID: 0x500]
    data.FloatBytes.f[0] = (float)uxTaskGetStackHighWaterMark(ContorlHandle);
    data.FloatBytes.f[1] = 0.0f;
    CAN_Send_Data(0x500, data.FloatBytes.bytes, 4);

    // 2. 发送 CanDataCenterTask 的高水位 [ID: 0x501]
    data.FloatBytes.f[0] = (float)uxTaskGetStackHighWaterMark(CanDataCenterHandle);
    data.FloatBytes.f[1] = 0.0f;
    CAN_Send_Data(0x501, data.FloatBytes.bytes, 4);

    // 3. 发送 EkfAlgorithmTask 的高水位 [ID: 0x502]
    data.FloatBytes.f[0] = (float)uxTaskGetStackHighWaterMark(EkfAlgorithmHandle);
    data.FloatBytes.f[1] = 0.0f;
    CAN_Send_Data(0x502, data.FloatBytes.bytes, 4);

    // 4. 发送 PowerManagementTask 的高水位 [ID: 0x503]
    data.FloatBytes.f[0] = (float)uxTaskGetStackHighWaterMark(PowerManagementHandle);
    data.FloatBytes.f[1] = 0.0f;
    CAN_Send_Data(0x503, data.FloatBytes.bytes, 4);
    
    /* 如果需要，收集并通过 USB 发送运行时统计信息（来自 motion control 的最近执行耗时） */
    char *runTimeStats = pvPortMalloc(1024); // 动态分配 1KB
    if (runTimeStats != NULL) {
        memset(runTimeStats, 0, 1024);
        vTaskGetRunTimeStats(runTimeStats);

        /* 仅发送统计体（不包含 MotionControl 精确耗时） */
        CDC_Transmit_FS((uint8_t *)runTimeStats, strlen(runTimeStats));

        vPortFree(runTimeStats);
    }
  }
}
/* USER CODE END Application */

