#include "LED.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 1. 开启 GPIOA 时钟和 AFIO 辅助功能时钟 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	/* 2. 禁用JTAG，保留SWD，释放 PA15 (JTDI) 作为普通引脚使用 */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	/* 3. 设置默认输出高电平 (LED默认熄灭，假设是低电平点亮) */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_SET);

	/* 4. 配置 GPIO 引脚模式 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
	GPIO_InitStruct.Pull = GPIO_NOPULL;           // 无上下拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 50MHz高速
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void LED1_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void LED1_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void LED1_Turn(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); // HAL库自带引脚翻转函数，一行搞定
}

void LED2_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

void LED2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

void LED2_Turn(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
}

void LED7_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void LED7_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

void LED7_Turn(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
}

void LED8_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void LED8_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void LED8_Turn(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
}
