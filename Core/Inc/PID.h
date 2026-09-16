#ifndef __PID_H
#define __PID_H

#include "main.h"

/* 定义一个 PID 结构体，方便我们为 4 个轮子独立创建 4 个 PID 控制器 */
typedef struct {
    float Target_Val;     // 目标速度（我们期望轮子达到的速度）
    float Actual_Val;     // 实际速度（编码器测量出来的实际速度）
    float Err;            // 当前偏差 (Target - Actual)
    float Err_Last;       // 上一次的偏差
    float Err_Prev;       // 上上次的偏差
    float Kp, Ki, Kd;     // PID 的三个核心调校参数
    float Output;         // PID 计算后的输出值（直接赋给定时器的 PWM 占空比）
} PID_TypeDef;

/* 函数声明 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
float PID_Calculate(PID_TypeDef *pid, float target_val, float actual_val);

#endif
