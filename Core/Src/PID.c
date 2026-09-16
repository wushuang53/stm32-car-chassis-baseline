#include "PID.h"

/**
  * @brief  PID 初始化：为小车轮子设定初始的 Kp, Ki, Kd 参数
  */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Target_Val = 0.0f;
    pid->Actual_Val = 0.0f;
    pid->Err = 0.0f;
    pid->Err_Last = 0.0f;
    pid->Err_Prev = 0.0f;
    pid->Output = 0.0f;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
  * @brief  增量式 PID 算法实现（每隔固定的时间，比如 10ms，调用一次本函数）
  * @param  pid: 指向对应轮子 PID 结构体的指针
  * @param  target_val: 期望的目标转速
  * @param  actual_val: 编码器测得的实际转速
  * @retval 计算后应该输出给电机的 PWM 占空比值
  */
float PID_Calculate(PID_TypeDef *pid, float target_val, float actual_val)
{
    pid->Target_Val = target_val;
    pid->Actual_Val = actual_val;
    
    // 1. 计算当前的偏差
    pid->Err = pid->Target_Val - pid->Actual_Val;
    
    // 2. 增量式 PID 核心数学公式
    float increment = pid->Kp * (pid->Err - pid->Err_Last)
                    + pid->Ki * pid->Err
                    + pid->Kd * (pid->Err - 2.0f * pid->Err_Last + pid->Err_Prev);
                    
    // 3. 将增量累加到输出中
    pid->Output += increment;
    
    // 4. 占空比限幅保护：防止计算出的占空比超过定时器的最大值（假设 ARR 为 1000）
    if (pid->Output > 1000.0f)  pid->Output = 1000.0f;
    if (pid->Output < -1000.0f) pid->Output = -1000.0f; // 负数代表反转
    
    // 5. 更新历史偏差记录，留给下一次计算使用
    pid->Err_Prev = pid->Err_Last;
    pid->Err_Last = pid->Err;
    
    return pid->Output;
}
