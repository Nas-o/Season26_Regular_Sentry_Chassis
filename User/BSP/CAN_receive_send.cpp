/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief can bsp层发送与接受
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright Copyright (c) 2024
 *
 */
// #include "cover_headerfile_h.h"
#include "CAN_receive_send.hpp"
#include "BoardLink.hpp"
#include "motor.hpp"
#include "dm_imu.hpp"
#include "supercup.hpp"
#include "BoardLink.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"

// 全局变量：FIFO溢出标志�?(0:正常, 1:溢出)
volatile uint8_t CAN_FIFO_Overflow_Flag = 0;

// CAN寄存器及控制�?
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3; // 定义原型在fdcan.c文件



/**
 * @brief 获取指定CAN总线的句�?
 */
FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus) {
    switch (can_bus) {
        case 0: return &hfdcan1;
        case 1: return &hfdcan2;
        case 2: return &hfdcan3;
        default: return &hfdcan1;
    }
}

/**
 * @brief 初始化can,包含过滤器配置与使能
 *
 */
void Can_Init(void)
{
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索�?
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤�?关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 将上述配置到CAN1
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0);
  HAL_FDCAN_Start(&hfdcan3);
}

/**
************************************************************************
* @brief:      	Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数�?
************************************************************************
**/
uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据�?
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;          // 发送数据长�?
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指�?
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存�?
  TxHeader.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0�?xFF
  
  
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
    return 1; // 发�?
  return 0;
}

/**
************************************************************************
* @brief:      	Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓�?
* @retval:     	接收的数据长�?
* @details:    	接收数据
************************************************************************
**/
uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, fdcan_RxHeader, buf) != HAL_OK)
    return 0; // 接收数据
  return fdcan_RxHeader->DataLength >> 16;
}

/**
 * @brief  CAN接收回调函数 (FIFO0)
 *         按优先级依次匹配: 板间通信 → 超级电容 → IMU → DJI电机 → DM电机
 * @param  hfdcan     CAN外设句柄
 * @param  RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /* ---- FIFO溢出检测 ---- */
    if ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) != RESET)
    {
        CAN_FIFO_Overflow_Flag = 1;
    }
    else
    {
        CAN_FIFO_Overflow_Flag = 0;
    }

    /* ---- 取出帧数据 ---- */
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
        return;

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    uint16_t id = rx_header.Identifier;

    /* ---- 1. 板间通信 (路由表分发, 匹配即退出) ---- */
    if (BoardLink_RxDispatch(hfdcan, id, rx_data))
        return;

    /* ---- 2. 超级电容 ---- */
    if (id == Supercap_receive_id || id == Supercap_chassis_power_id)
    {
        Supercup_DecodeCandata(hfdcan, rx_data, id);
    }

    /* ---- 3. IMU ---- */
    if (id == imu.mst_id)
    {
        IMU_UpdateData(rx_data);
    }

    /* ---- 4. DJI电机 (内部按ID范围匹配) ---- */
    DJIMotor_DecodeCandata(hfdcan, id, rx_data);

    /* ---- 5. DM电机 (内部按ID范围匹配) ---- */
    DMMotor_DecodeCandata(hfdcan, id, rx_data);
}


/**
 * @brief CAN错误/状态变化回调函�?
 * @note  
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索�?
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤�?关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter); // 将上述配置到CAN
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(hfdcan);
}