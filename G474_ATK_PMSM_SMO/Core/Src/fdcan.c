/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

volatile uint32_t g_fdcan1_rx_count;        // 累计接收帧计数
volatile uint32_t g_fdcan1_last_rx_id;      // 最近一次接收到的ID
volatile uint8_t  g_fdcan1_last_rx_len;     // 最近一次接收数据长度(0~8)
volatile uint8_t  g_fdcan1_last_rx_data[8]; // 最近一次接收数据
volatile uint32_t g_fdcan1_error_count;     // 错误事件计数

/* 将字节长度转换为HAL定义的DLC值 */
static uint32_t FDCAN_LenToDlc(uint8_t len)
{
  switch (len)
  {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

/* 将DLC值反解成实际字节数(基础联调只处理0~8字节) */
static uint8_t FDCAN_DlcToLen(uint32_t dlc)
{
  if (dlc <= FDCAN_DLC_BYTES_8) return (uint8_t)dlc;
  return 8;
}

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 7;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 34;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  //配置过滤器、开中断、启动总线
  FDCAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.IdType = FDCAN_STANDARD_ID;             // 标准ID(11位)
  sFilterConfig.FilterIndex = 0;                        // 使用过滤器0
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // 掩码过滤
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 命中后进入RX FIFO0
  sFilterConfig.FilterID1 = 0x000;                      // 过滤ID
  sFilterConfig.FilterID2 = 0x000;                      // 掩码=0，全放行
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,          // 非匹配标准帧拒收
                                   FDCAN_REJECT,          // 非匹配扩展帧拒收
                                   FDCAN_REJECT_REMOTE,   // 标准远程帧拒收
                                   FDCAN_REJECT_REMOTE)   // 扩展远程帧拒收
      != HAL_OK) Error_Handler();

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE | // FIFO0接收新消息中断
                                     FDCAN_IT_BUS_OFF |              // 总线关闭事件
                                     FDCAN_IT_ERROR_WARNING |        // 错误告警
                                     FDCAN_IT_ERROR_PASSIVE,         // 错误被动
                                     0) != HAL_OK) Error_Handler();

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler(); // 启动FDCAN，必须调用
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* 发送标准数据帧(11位ID, 0~8字节) */
HAL_StatusTypeDef FDCAN1_SendStd(uint16_t id, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  uint8_t i = 0;

  if (len > 8U) len = 8U;

  txHeader.Identifier = ((uint32_t)id & 0x7FFU);   // 限制为11位ID
  txHeader.IdType = FDCAN_STANDARD_ID;             // 标准帧ID
  txHeader.TxFrameType = FDCAN_DATA_FRAME;         // 数据帧
  txHeader.DataLength = FDCAN_LenToDlc(len);       // 数据长度
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;          // 关闭速率切换(经典CAN)
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;           // 经典CAN格式
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 不存Tx事件
  txHeader.MessageMarker = 0;

  for (i = 0; i < len; i++) txData[i] = data[i];

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
}

/* 接收回调RX FIFO0：收到一帧就取出来并缓存 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};
  uint8_t rxData[8] = {0};
  uint8_t i = 0;

  if (hfdcan != &hfdcan1) return;
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    g_fdcan1_error_count++; // 读消息失败记错
    return;
  }

  g_fdcan1_rx_count++;                              // 接收总数+1
  g_fdcan1_last_rx_id = rxHeader.Identifier;        // 记录最近ID
  g_fdcan1_last_rx_len = FDCAN_DlcToLen(rxHeader.DataLength); // 记录最近长度

  for (i = 0; i < g_fdcan1_last_rx_len; i++)
  {
    g_fdcan1_last_rx_data[i] = rxData[i];          // 保存最近接收数据
  }
}

/* 错误状态回调：用于联调阶段统计错误事件 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  UNUSED(ErrorStatusITs);
  if (hfdcan == &hfdcan1) g_fdcan1_error_count++;
}





/* USER CODE END 1 */
