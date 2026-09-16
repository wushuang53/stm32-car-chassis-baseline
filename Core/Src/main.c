/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "sensor.h"
#include "LED.h"
#include "usart.h"     
#include <string.h>
#include "PID.h"
#include "avoid.h"
#include "HC_SR04.h"
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

/* USER CODE BEGIN PV */
uint8_t SensorState[4];
uint8_t Area, Last_Area; // 巡线区域记录变量

// 定义小车运行模式枚举
typedef enum {
    MODE_PATROL = 0,    // 循迹巡逻模式
    MODE_INTERCEPT = 1, // 离轨拦截模式
} CarMode_TypeDef;

CarMode_TypeDef CarMode = MODE_PATROL; // 默认开机为巡逻模式
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Send_Status_To_Upper(float distance_mm, uint8_t is_emergency);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	LED_Init(); 
  Motor_Init();   //启动PWM通道
	Usart3_Init(); 
	Avoid_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// 1. 判断串口 3 是否收到了来自电脑端发送的指令
//		if (Usart3_GetRxFlag()) 
//		{
//			if (strcmp(Rx_Packet, "CAR_F") == 0)
//			{
//				Car_Forward(200);   // 直行追踪
//			}
//			else if (strcmp(Rx_Packet, "CAR_B") == 0)
//			{
//				Car_Backward(200);
//			}
//			else if (strcmp(Rx_Packet, "CAR_TF") == 0)
//			{
//				Car_TurnLeft(200);  // 左偏，左转追踪
//			}
//			else if (strcmp(Rx_Packet, "CAR_TR") == 0)
//			{
//				Car_TurnRight(200); // 右偏，右转追踪
//			}
//			else if (strcmp(Rx_Packet, "CAR_TSF") == 0)
//			{
//				Car_TransLeft(200);
//			}
//			else if (strcmp(Rx_Packet, "CAR_TSR") == 0)
//			{
//				Car_TransRight(200);
//			}
//			else
//			{
//				Car_Stop();         // 停止
//			}
//		}

//		HAL_Delay(30);  // 循环执行延时 (30ms 足够响应)

  // 1. 高频刷新超声波避障检测，并获取当前的物理状态
		Avoid_Check_Obstacle();
		uint8_t is_emergency = Avoid_Get_Emergency_Flag();
		float distance_mm = Avoid_Get_Distance();

		// 2. 监测串口 3 是否收到了上位机的 8 字节十六进制指令
		if (Usart3_GetRxFlag()) 
		{
			uint8_t cmd_type = RxBuffer[2]; 
			
			if (cmd_type == 0x01) // 拦截指令
			{
				// ⚠️ 只有在没有触发紧急防撞时，才允许切入拦截！
				if (is_emergency == 0)
				{
					CarMode = MODE_INTERCEPT; // 切换为：主动拦截模式！
					
					int8_t rx_vx = (int8_t)RxBuffer[3];
					int8_t rx_vy = (int8_t)RxBuffer[4];
					int8_t rx_w  = (int8_t)RxBuffer[5];
					
					int16_t target_vx = (int16_t)rx_vx * 10;
					int16_t target_vy = (int16_t)rx_vy * 10;
					int16_t target_w  = (int16_t)rx_w * 10;
					
					Mechanics_Move(target_vx, target_vy, target_w);
				}
			}
			else if (cmd_type == 0x00) // 恢复巡逻
			{
				CarMode = MODE_PATROL;    // 切换回：循迹巡逻模式！
			}
		}

		// 3. 根据当前模式，执行不同的业务逻辑
		if (CarMode == MODE_PATROL)
		{
			// ⚠️ 只有在没有触发紧急防撞时，才允许循迹巡逻！
			if (is_emergency == 0)
			{
				SensorState[0] = Sensor0_Get_State();
				SensorState[1] = Sensor1_Get_State();
				SensorState[2] = Sensor2_Get_State();
				SensorState[3] = Sensor3_Get_State();
				
				if (SensorState[0] && !SensorState[1] && !SensorState[2] && SensorState[3]) 
				{
					Area = 3; Car_Forward(200);
				}
				else if (SensorState[0] && SensorState[1] && !SensorState[2] && SensorState[3]) 
				{
					Area = 4; Car_TurnRight(200);
				}
				else if (SensorState[0] && SensorState[1] && SensorState[2] && !SensorState[3]) 
				{
					Area = 6; Car_TurnRight(300);
				}
				else if (SensorState[0] && !SensorState[1] && SensorState[2] && SensorState[3]) 
				{
					Area = 2; Car_TurnLeft(200);
				}
				else if (!SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3]) 
				{
					Area = 0; Car_TurnLeft(300);
				}				
				else if (SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3])  
				{
					if (Last_Area == 4)      { Area = 5; Car_TurnRight(250); }
					else if (Last_Area == 2) { Area = 1; Car_TurnLeft(250); }
				}
				Last_Area = Area;
			}
		}

		// 4. 每隔 1 秒，自动向电脑（林志涛）上报一次小车传感器状态
		static uint32_t last_time = 0;
		if (HAL_GetTick() - last_time >= 1000)
		{
			Send_Status_To_Upper(distance_mm, is_emergency); // 调用状态上报函数
			last_time = HAL_GetTick();
		}

		HAL_Delay(30); 
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  下位机状态主动上报函数（STM32 -> Radxa/PC）[Page 8]
  */
void Send_Status_To_Upper(float distance_mm, uint8_t is_emergency)
{
	uint16_t dist = (uint16_t)distance_mm;
	uint8_t dist_h = (dist >> 8) & 0xFF; 
	uint8_t dist_l = dist & 0xFF;        
	
	uint8_t tx_frame[8] = {0xFF, 0xFE, 0x02, dist_h, dist_l, is_emergency, 0x00, 0xAA};
	Usart3_SendArray(tx_frame, 8);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
