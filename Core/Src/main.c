/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Mecanum Wheel Car + USART1 Integration)
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
#include <string.h>
#include "PID.h"
#include "avoid.h"
#include "HC_SR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 定义小车运行模式枚举
typedef enum {
    MODE_PATROL = 0,    // 循迹巡逻模式
    MODE_INTERCEPT = 1, // 离轨拦截模式
} CarMode_TypeDef;
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

CarMode_TypeDef CarMode = MODE_PATROL; // 默认开机为巡逻模式
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Send_Status_To_Upper(float distance_mm, uint8_t is_emergency);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_Init(); 
  Motor_Init();   // 启动电机 PWM 通道
  Usart1_Init();  // 启动串口 1 中断接收
  Avoid_Init();   // 避障初始化
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. 高频刷新超声波避障检测，并获取当前的物理状态
    Avoid_Check_Obstacle();
    uint8_t is_emergency = Avoid_Get_Emergency_Flag();
    float distance_mm = Avoid_Get_Distance();

    // 实时读取4路红外状态 (0代表黑线，1代表白地)
    SensorState[0] = Sensor0_Get_State(); // PB1 左外
    SensorState[1] = Sensor1_Get_State(); // PB0 左内
    SensorState[2] = Sensor2_Get_State(); // PB4 右内
    SensorState[3] = Sensor3_Get_State(); // PB5 右外
    
    // 只要有任意一路红外探头检测到黑线，hit_black_line 为 1
    uint8_t hit_black_line = (!SensorState[0] || !SensorState[1] || !SensorState[2] || !SensorState[3]);

    // 2. 监测串口 1 是否收到了上位机的 8 字节十六进制指令
    if (Usart1_GetRxFlag()) 
    {
        uint8_t cmd_type = RxBuffer[2]; 
        
        if (cmd_type == 0x01) // 拦截 / 上位机运动控制指令
        {
            int8_t rx_vx = (int8_t)RxBuffer[3];
            int8_t rx_vy = (int8_t)RxBuffer[4];
            int8_t rx_w  = (int8_t)RxBuffer[5];

            // =================================================================
            // 核心回航自锁：如果上位机发的是倒车指令 (rx_vx < 0)，且探头压到了黑线：
            // 单片机立刻就地截断倒车，强制切回巡线模式，拒绝继续往后倒！
            // 追人时 rx_vx > 0，即使碾过黑线也绝不误触发，继续全速追人！
            // =================================================================
            if (rx_vx < 0 && hit_black_line)
            {
                CarMode = MODE_PATROL; // 触线自锁，立刻交还巡线！
                Last_Area = 3;
            }
            else
            {
                CarMode = MODE_INTERCEPT; // 正常上位机接管 (追人冲锋或在白地上倒车)
                int16_t target_vx = (int16_t)rx_vx * 10;
                int16_t target_vy = (int16_t)rx_vy * 10;
                int16_t target_w  = (int16_t)rx_w * 10;
                Mechanics_Move(target_vx, target_vy, target_w);
            }
        }
        else if (cmd_type == 0x00) // 恢复自主巡逻模式
        {
            CarMode = MODE_PATROL; 
            Last_Area = 3;
        }
    }

    // 2.1 双重保险：如果在接管模式倒车中途，探头刚刚划过黑线，立即毫秒级切换！
    if (CarMode == MODE_INTERCEPT)
    {
        int8_t current_rx_vx = (int8_t)RxBuffer[3];
        if (current_rx_vx < 0 && hit_black_line)
        {
            CarMode = MODE_PATROL;
            Last_Area = 3;
        }
    }

    // 3. 根据当前模式，执行不同的业务逻辑 (100% 恢复最初原始速度！)
    if (CarMode == MODE_PATROL)
    {
        // 循迹巡逻模式下的避障与巡线逻辑
        if (is_emergency == 0)
        {
            if (SensorState[0] && !SensorState[1] && !SensorState[2] && SensorState[3])   // Area 3: 居中，直行
            {
                Area = 3;
                Car_Forward(200);
            }
            else if (SensorState[0] && SensorState[1] && !SensorState[2] && SensorState[3]) // Area 4: 轻微偏左，向右小转修正
            {
                Area = 4;
                Car_TurnRight(200);
            }
            else if (SensorState[0] && SensorState[1] && SensorState[2] && !SensorState[3]) // Area 6: 严重偏左，向右大转急转
            {
                Area = 6;
                Car_TurnRight(300);
            }
            else if (SensorState[0] && !SensorState[1] && SensorState[2] && SensorState[3]) // Area 2: 轻微偏右，向左小转修正
            {
                Area = 2;
                Car_TurnLeft(200);
            }
            else if (!SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3]) // Area 0: 严重偏右，向左大转急转
            {
                Area = 0;
                Car_TurnLeft(300);
            }				
            else if (SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3])  // 全部脱轨丢失黑线
            {
                if (Last_Area == 4)   // 根据上一次的转向趋势执行惯性寻线
                {
                    Area = 5;
                    Car_TurnRight(250);
                }
                else if (Last_Area == 2)  
                {
                    Area = 1;
                    Car_TurnLeft(250);
                }
            }

            Last_Area = Area;
        }
    }

    // 4. 每隔 1 秒，自动向上位机上报一次小车传感器状态 (0xFF 0xFE 0x02 ...)
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time >= 1000)
    {
        Send_Status_To_Upper(distance_mm, is_emergency);
        last_time = HAL_GetTick();
    }

    HAL_Delay(30); // 恢复最初原版的 30ms 延时
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
  * @brief  下位机状态主动上报函数（STM32 -> Radxa/PC）
  */
void Send_Status_To_Upper(float distance_mm, uint8_t is_emergency)
{
    uint16_t dist = (uint16_t)distance_mm;
    uint8_t dist_h = (dist >> 8) & 0xFF; 
    uint8_t dist_l = dist & 0xFF;        
    
    uint8_t tx_frame[8] = {0xFF, 0xFE, 0x02, dist_h, dist_l, is_emergency, 0x00, 0xAA};
    Usart1_SendArray(tx_frame, 8); // 已修正为通过串口 1 发送
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
