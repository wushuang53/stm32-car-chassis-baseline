#include "Motor.h"

/* 引用 CubeMX 自动生成的定时器句柄 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

/**
  * @brief  电机初始化：启动 TIM2 与 TIM4 的 8 路 PWM 输出
  */
void Motor_Init(void)
{
    // 启动 TIM2 的 CH1 ~ CH4 (PA0, PA1, PA2, PA3)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // 启动 TIM4 的 CH1 ~ CH4 (PB6, PB7, PB8, PB9)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/******************** 基础整车运动控制 ********************/

void Car_Forward(uint16_t Speed)
{
    Motor1_SetSpeed(1, Speed);
    Motor2_SetSpeed(1, Speed);
    Motor3_SetSpeed(1, Speed);
    Motor4_SetSpeed(1, Speed);
}

void Car_Backward(uint16_t Speed)
{
    Motor1_SetSpeed(0, Speed);
    Motor2_SetSpeed(0, Speed);
    Motor3_SetSpeed(0, Speed);
    Motor4_SetSpeed(0, Speed);
}

void Car_TurnLeft(uint16_t Speed)
{
    Motor1_SetSpeed(0, Speed);
    Motor2_SetSpeed(1, Speed);
    Motor3_SetSpeed(1, Speed);
    Motor4_SetSpeed(0, Speed);
}

void Car_TurnRight(uint16_t Speed)
{
    Motor1_SetSpeed(1, Speed);
    Motor2_SetSpeed(0, Speed);
    Motor3_SetSpeed(0, Speed);
    Motor4_SetSpeed(1, Speed);
}

void Car_TransLeft(uint16_t Speed)
{
    Motor1_SetSpeed(1, Speed);
    Motor2_SetSpeed(0, Speed);
    Motor3_SetSpeed(1, Speed);
    Motor4_SetSpeed(0, Speed);
}

void Car_TransRight(uint16_t Speed)
{
    Motor1_SetSpeed(0, Speed);
    Motor2_SetSpeed(1, Speed);
    Motor3_SetSpeed(0, Speed);
    Motor4_SetSpeed(1, Speed);
}

void Car_Stop(void)
{
    Motor1_SetSpeed(0, 0);
    Motor2_SetSpeed(0, 0);
    Motor3_SetSpeed(0, 0);
    Motor4_SetSpeed(0, 0);
}

// 单个电机底层调速

// PA0 -> TIM2_CH1 -> MOTOR1A (Backward)
// PA1 -> TIM2_CH2 -> MOTOR1B (Forward)
//motor1（左下）的正转方向为视觉上的反转 即Dir = 0;
void Motor1_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000;
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Speed);
    }
}

// PA2 -> TIM2_CH3 -> MOTOR2A (Backward)
// PA3 -> TIM2_CH4 -> MOTOR2B (Forward)
//motor2（右下）的正转方向为视觉上的正转 即Dir = 1;
void Motor2_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000;
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Speed);
    }
}

// PB6 -> TIM4_CH1 -> MOTOR3A(OUT4A) (Backward)
// PB7 -> TIM4_CH2 -> MOTOR3B(OUT4B) (Forward)
//motor3（右上）的正转方向为视觉上的正转 即Dir = 1;
void Motor3_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000;
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Speed);
    }
}

// PB8 -> TIM4_CH3 -> MOTOR4A(OUT3A) (Backward)
// PB9 -> TIM4_CH4 -> MOTOR4B(OUT3B) (Forward)
//motor4（左上）的正转方向为视觉上的反转 即Dir = 0;
void Motor4_SetSpeed(uint8_t Dir, uint16_t Speed)
{
    if (Speed >= 1000) Speed = 1000;
    if (Dir)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Speed); // 修复原代码硬编码500的Bug
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Speed); // 修复原代码硬编码500的Bug
    }
}

/**
  * @brief  麦克纳姆轮全向运动解算函数（1:1完美匹配你的电机接线与朝向）
  * @param  Vx: 期望的横向平移速度（往右为正，往左为负，范围 -1000 ~ 1000）
  * @param  Vy: 期望的前后直行速度（向前为正，向后为负，范围 -1000 ~ 1000）
  * @param  W:  期望的自转角速度（顺时针为正，逆时针为负，范围 -1000 ~ 1000）
  */
void Mechanics_Move(int16_t Vx, int16_t Vy, int16_t W)
{
    // 1. 麦克纳姆轮物理公式，直接分解出 4 个电机各自需要达到的目标速度（含正负）
    int16_t Target_Speed1 = Vy - Vx - W; // Motor1 左后
    int16_t Target_Speed2 = Vy + Vx + W; // Motor2 右后
    int16_t Target_Speed3 = Vy - Vx + W; // Motor3 右前
    int16_t Target_Speed4 = Vy + Vx - W; // Motor4 左前

    // 2. 依次控制 4 个电机的正反转方向和 PWM 占空比输出
    
    // 控制电机 1 (左后)
    if (Target_Speed1 >= 0) Motor1_SetSpeed(1, Target_Speed1);
    else                    Motor1_SetSpeed(0, -Target_Speed1); // 速度为负时，反转，速度取绝对值

    // 控制电机 2 (右后)
    if (Target_Speed2 >= 0) Motor2_SetSpeed(1, Target_Speed2);
    else                    Motor2_SetSpeed(0, -Target_Speed2);

    // 控制电机 3 (右前)
    if (Target_Speed3 >= 0) Motor3_SetSpeed(1, Target_Speed3);
    else                    Motor3_SetSpeed(0, -Target_Speed3);

    // 控制电机 4 (左前)
    if (Target_Speed4 >= 0) Motor4_SetSpeed(1, Target_Speed4);
    else                    Motor4_SetSpeed(0, -Target_Speed4);
}
