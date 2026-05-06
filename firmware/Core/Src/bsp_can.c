#include <string.h>
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

/* 在 freertos.c 中实现的微秒延时函数 */
extern void delay_us(uint32_t us);



// 注意：canRxQueueHandle 的实际定义放在了 freertos.c 中
// 我们这里只是从 bsp_can.h 里面通过 extern 引用它

/**
 * @brief  配置 CAN1 的接收过滤器
 * @note   当前配置为“接收所有报文”的掩码模式，适用于单板调试初期
 */
void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                      // 过滤器组0 (如果不使用CAN2，选0即可)
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位宽
    sFilterConfig.FilterIdHigh = 0x0000;               // ID高位
    sFilterConfig.FilterIdLow = 0x0000;                // ID低位
    sFilterConfig.FilterMaskIdHigh = 0x0000;           // 掩码高位(全为0表示不关心，全部放行)
    sFilterConfig.FilterMaskIdLow = 0x0000;            // 掩码低位
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 绑定到FIFO0接收
    sFilterConfig.FilterActivation = ENABLE;           // 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;           // CAN2的起始过滤器组(STM32F4固定分配)

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler(); // 配置失败则进入死循环
    }
}

/**
 * @brief  向 CAN 总线发送标准帧数据
 * @param  id   报文的标准ID (如 0x123)
 * @param  data 要发送的数据数组指针
 * @param  len  发送的数据长度 (0~8字节)
 */
void CAN_Send_Data(uint16_t id, uint8_t *data, uint8_t len)
{
    CAN_TxPacketTypeDef txPacket;

    txPacket.header.StdId = id;                   // 标准报文ID
    txPacket.header.ExtId = 0x00;                 // 扩展报文ID（一般不填）
    txPacket.header.RTR = CAN_RTR_DATA;           // 发送数据帧
    txPacket.header.IDE = CAN_ID_STD;             // 使用标准ID类型（11位）
    txPacket.header.DLC = len;                    // 数据长度（0-8字节）
    txPacket.header.TransmitGlobalTime = DISABLE; // 不发送时间戳

    // 拷贝数据（保证8字节缓冲区内）
    memset(txPacket.data, 0, sizeof(txPacket.data));
    if (len > 8) len = 8;
    memcpy(txPacket.data, data, len);

    // 将要发送的报文放入 canTxQueue，由 CAN_TxTask 实际发送
    if (canTxQueueHandle != NULL) {
        osMessageQueuePut(canTxQueueHandle, &txPacket, 0, 0);
    }
}

/* CAN Tx task: 从队列取出报文并调用 HAL 发送 */
void CAN_TxTask(void *argument)
{
    CAN_TxPacketTypeDef txPacket;
    uint32_t TxMailbox;

    for(;;)
    {

        if (osMessageQueueGet(canTxQueueHandle, &txPacket, NULL, osWaitForever) == osOK)
        {
            uint8_t count = 0;
            while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 && count < 10) // 最多等待10次，每次100us，总共1ms超时
            {
                osDelay(1); // 等待1ms，直到有空闲邮箱
                count++;
            }
            if (count >= 10 || HAL_CAN_AddTxMessage(&hcan1, &txPacket.header, txPacket.data, &TxMailbox) != HAL_OK)
            {
                // 发送失败直接丢弃，不要进入死循环卡死系统
                continue;
            }
        }
    }
}

/**
 * @brief CAN1 FIFO0 接收中断回调函数
 * @note  这个函数由 HAL 库在中断上下文中调用，绝不可在此处进行延时或阻塞操作
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxPacketTypeDef rxPacket;

    if (hcan->Instance == CAN1)
    {
        // 1. 从 FIFO0 邮箱中读取接收到的报文
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxPacket.header, rxPacket.data) == HAL_OK)
        {
            // 2. 将整个结构体推入 FreeRTOS 队列，给另外的任务慢慢处理
            // 注意：因为这是在中断服务程序(ISR)里，所以必须使用 FromISR 或者系统封装好的底层 API
            if(canRxQueueHandle != NULL) 
            {
                // 在 CMSIS-OS V2 中，osMessageQueuePut 内部已经做了 ISR 级别判断，可以直接用！
                // 只是 timeout 必须设为 0
                osMessageQueuePut(canRxQueueHandle, &rxPacket, 0, 0); 
            }
        }
    }
}
