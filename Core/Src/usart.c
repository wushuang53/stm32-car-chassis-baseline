/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#pragma import(__use_no_semihosting)  // 关闭半主机
#include <stdio.h>

extern UART_HandleTypeDef huart3;     // 声明串口 3 句柄

struct __FILE
{
	int handle; 
};

FILE __stdout;
FILE __stdin;

/* 重定向 printf 到串口 3 */
int fputc(int ch, FILE *f) 
{
	Usart3_SendByte(ch);
	return ch;
}

void _sys_exit(int x) 
{
	x = x; 
}

uint8_t  Rx_Data;
uint8_t Rx_Flag;
uint8_t RxState = 0;       // 状态机状态变量
uint8_t RxBuffer[8];       // 存储收到的 8 字节完整数据包
uint8_t temp_buffer[8];    // 临时接收缓冲区

void Usart3_Init(void)
{
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&Rx_Data, 1);
}

/**
  * @brief  获取串口3接收完成标志位
  * @param  None
  * @retval 1: 接收完成并清除标志位, 0: 未接收完成
  */
uint8_t Usart3_GetRxFlag(void)
{
    if (Rx_Flag == 1)
    {
        Rx_Flag = 0; // 自动清零标志位，避免 main.c 中的 while 循环重复处理同一包数据
        return 1;
    }
    return 0;
}

void Usart3_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart3, &Byte, 1, HAL_MAX_DELAY);
}

void Usart3_SendArray(uint8_t *Array, uint16_t Length)
{
	HAL_UART_Transmit(&huart3, Array, Length, HAL_MAX_DELAY);
}

void Usart3_SendString(char *String)
{
	uint8_t i;
	for(i = 0; String[i] != '\0'; i++)
	{
		Usart3_SendByte(String[i]);
	}
}

// 串口 3 的接收中断回调函数（全新 8 字节状态机解析）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		static uint8_t byte_count = 0;
		
		if (RxState == 0) // 1. 寻找第一帧头 0xFF
		{
			if (Rx_Data == 0xFF)
			{
				RxState = 1;
				temp_buffer[0] = Rx_Data;
			}
		}
		else if (RxState == 1) // 2. 寻找第二帧头 0xFE
		{
			if (Rx_Data == 0xFE)
			{
				RxState = 2;
				temp_buffer[1] = Rx_Data;
				byte_count = 2; // 已收到2个字节
			}
			else
			{
				RxState = 0; // 帧头不对，重置状态机
			}
		}
		else if (RxState == 2) // 3. 接收中间的有效载荷数据（第2字节到第6字节）
		{
			temp_buffer[byte_count++] = Rx_Data;
			if (byte_count == 7) // 已经收到了 7 个字节
			{
				RxState = 3;
			}
		}
		else if (RxState == 3) // 4. 寻找帧尾 0xAA
		{
			if (Rx_Data == 0xAA)
			{
				temp_buffer[7] = Rx_Data;
				
				// 靠帧头帧尾过滤错位数据成功，将临时数据拷入全局缓冲区
				for (uint8_t i = 0; i < 8; i++)
				{
					RxBuffer[i] = temp_buffer[i];
				}
				Rx_Flag = 1; // 标记一包数据接收完成
			}
			RxState = 0; // 重置状态机，准备接收下一包
			byte_count = 0;
		}
		
		// 重新开启接收中断
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Rx_Data, 1);
	}
}
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
