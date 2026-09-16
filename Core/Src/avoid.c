#include "Avoid.h"
#include "HC_SR04.h"
#include "motor.h"
#include "LED.h"

static uint8_t Is_Emergency = 0; // 内部紧急避障标志位
static float Distance_mm = 0.0f; // 内部实时距离缓存 (mm)

/**
  * @brief  初始化避障模块：包含底层超声波定时器的初始化
  */
void Avoid_Init(void)
{
    HC_SR04Init(); // 调用底层的超声波硬件初始化
    Is_Emergency = 0;
    Distance_mm = 0.0f;
}

/**
  * @brief  核心避障逻辑检测（每 30ms 调用一次）
  */
void Avoid_Check_Obstacle(void)
{
    Distance_mm = Sonar(); // 1. 读取超声波探头的物理距离 (mm)
    
    // 2. 物理防撞保底安全锁：如果前方距离大于 0 且小于 15cm (150mm)
    if (Distance_mm > 0.0f && Distance_mm < 150.0f)
    {
        Is_Emergency = 1; // 标记触发紧急刹车
        Car_Stop();       // 强行制动，直接拉死轮子占空比！
        LED1_ON();        // 开启红灯报警
    }
    else
    {
        Is_Emergency = 0; // 恢复正常
        LED1_OFF();       // 关闭红灯报警
    }
}

/**
  * @brief  获取当前的紧急刹车标志位（供 main.c 决策读取）
  */
uint8_t Avoid_Get_Emergency_Flag(void)
{
    return Is_Emergency;
}

/**
  * @brief  获取当前测得的实际距离（供 main.c 上报使用）
  */
float Avoid_Get_Distance(void)
{
    return Distance_mm;
}
