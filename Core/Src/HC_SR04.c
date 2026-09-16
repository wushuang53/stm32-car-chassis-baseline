#include "stm32f1xx_hal.h"
#include "HC_SR04.h"

extern TIM_HandleTypeDef htim3;
volatile uint16_t TimeCount; // 告诉编译器该变量会在中断中被修改

/* 维持原有的微秒级延时函数 */
static void HC_SR04_DelayUs(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 8;
    while (ticks--)
    {
        __NOP();
    }
}

// Trig PB15  
// Echo PC15
void HC_SR04Init(void)
{
	HAL_TIM_Base_Start_IT(&htim3);
}

float Sonar(void)
{
    float Distance = 0, Distance_mm = 0; 
    uint32_t Time_end;
    uint32_t timeout; // 引入超时计数器
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  
    HC_SR04_DelayUs(15);                                  
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
    
    // 1. 等待 Echo 变高，加入超时限制
    timeout = 100000; // 根据主频估算的一个安全限制
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
    {
        if (--timeout == 0) return 0.0f; // 超时直接退出，返回0
    }
    
    TimeCount = 0;
    
    // 2. 等待 Echo 变低，加入超时限制
    timeout = 100000;
    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET)
    {
        if (--timeout == 0) return 0.0f; // 超时直接退出
    }
    
    Time_end = TimeCount * 10;
    
    if (Time_end < 24000)
    {
        Distance = Time_end * 0.000001f * 340 / 2;
        Distance_mm = Distance * 1000;
    }
    return Distance_mm;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		TimeCount++;
	}
}
