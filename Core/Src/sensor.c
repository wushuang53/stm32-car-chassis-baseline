#include "Sensor.h"

/* 
 * 传感器引脚说明：
 * SENSOR0 -> PB1
 * SENSOR1 -> PB0
 * SENSOR2 -> PB4
 * SENSOR3 -> PB5
 * (初始化配置由 CubeMX 自动在 main.c 的 MX_GPIO_Init 中完成)
 */

uint8_t Sensor0_Get_State(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

uint8_t Sensor1_Get_State(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
}

uint8_t Sensor2_Get_State(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

uint8_t Sensor3_Get_State(void)
{
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}
