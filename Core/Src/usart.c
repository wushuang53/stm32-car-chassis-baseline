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
#pragma import(__use_no_semihosting)
#include <stdio.h>

extern UART_HandleTypeDef huart1;     // 声明串口 1 句柄

struct __FILE
{
	int handle; 
};

FILE __stdout;
FILE __stdin;

/* 重定向 printf 到串口 1 */
int fputc(int ch, FILE *f) 
{
	Usart1_SendByte(ch);
	return ch;
}

void _sys_exit(int x) 
{
	x = x; 
}

uint8_t  Rx_Data;
uint8_t  Rx_Flag;
uint8_t  RxState = 0;       // 状态机状态变量
uint8_t  RxBuffer[8];       // 存储收到的 8 字节完整数据包
uint8_t  temp_buffer[8];    // 临时接收缓冲区

void Usart1_Init(void)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_Data, 1);
}

/**
  * @brief  获取串口1接收完成标志位
  * @retval 1: 接收完成并清除标志位, 0: 未接收完成
  */
uint8_t Usart1_GetRxFlag(void)
{
    if (Rx_Flag == 1)
    {
        Rx_Flag = 0; // 自动清零标志位
        return 1;
    }
    return 0;
}

void Usart1_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}

void Usart1_SendArray(uint8_t *Array, uint16_t Length)
{
	HAL_UART_Transmit(&huart1, Array, Length, HAL_MAX_DELAY);
}

void Usart1_SendString(char *String)
{
	uint8_t i;
	for(i = 0; String[i] != '\0'; i++)
	{
		Usart1_SendByte(String[i]);
	}
}

// 串口 1 的接收中断回调函数（8 字节状态机解析）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
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
				byte_count = 2;
			}
			else
			{
				RxState = 0;
			}
		}
		else if (RxState == 2) // 3. 接收中间的有效载荷数据（第2到第6字节）
		{
			temp_buffer[byte_count++] = Rx_Data;
			if (byte_count == 7)
			{
				RxState = 3;
			}
		}
		else if (RxState == 3) // 4. 寻找帧尾 0xAA
		{
			if (Rx_Data == 0xAA)
			{
				temp_buffer[7] = Rx_Data;
				for (uint8_t i = 0; i < 8; i++)
				{
					RxBuffer[i] = temp_buffer[i];
				}
				Rx_Flag = 1; // 标记一包数据接收完成
			}
			RxState = 0;
			byte_count = 0;
		}
		
		// 重新开启接收中断
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_Data, 1);
	}
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
