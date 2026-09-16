#include "key.h"

void Key_Init(void)
{
    // CubeMX 已经在 MX_GPIO_Init() 中自动完成了引脚初始化
}

uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;
    
    // 按键 1 (PB0)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        HAL_Delay(20); // 消抖
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 1;
    }
    // 按键 2 (PB1)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 2;
    }
    // 按键 3 (PB4)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 3;
    }
    // 按键 4 (PB5)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 4;
    }    
    return KeyNum;
}
