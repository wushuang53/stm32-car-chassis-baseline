#include "Remote.h"

extern TIM_HandleTypeDef htim3;

uint8_t IR_State;
uint16_t IR_Time;
uint8_t IR_RepeatFlag, IR_DataFlag;
uint8_t IR_DATA[4];
uint8_t IR_pData;
uint8_t IR_Address, IR_Command;

void Remote_Init(void)
{
    /* 开启 TIM3 的通道 1 输入捕获中断 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

uint8_t IR_GetDataFlag(void)
{
    if (IR_DataFlag)
    {
        IR_DataFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t IR_GetRepeatFlag(void)
{
    if (IR_RepeatFlag)
    {
        IR_RepeatFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t IR_GetAddress(void)
{
    return IR_Address;
}

uint8_t IR_GetCommand(void)
{
    return IR_Command;
}

/* HAL 库定时器输入捕获中断统一回调函数 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        uint8_t Addr_Temp, Cmd_Temp;
        
        if (IR_State == 0)     // 空闲态
        {
            __HAL_TIM_SET_COUNTER(htim, 0); // 清零计数器
            IR_State = 1;
        }
        else if (IR_State == 1) // 准备态
        {
            IR_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 读取捕获值
            
            if (IR_Time > 13500 - 500 && IR_Time < 13500 + 500)  // Start信号 (13.5ms)
            {
                IR_State = 2;
            }
            else if (IR_Time > 11250 - 500 && IR_Time < 11250 + 500)  // Repeat信号 (11.25ms)
            {
                IR_State = 0;
                IR_RepeatFlag = 1;
            }
            else
            {
                IR_State = 1;
            }
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
        else if (IR_State == 2) // 接收态
        {
            IR_Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            if (IR_Time > 1120 - 500 && IR_Time < 1120 + 500)   // 逻辑0 (1.12ms)
            {
                IR_DATA[IR_pData / 8] &= ~(0x01 << (IR_pData % 8));
                IR_pData++;
            }
            else if (IR_Time > 2250 - 500 && IR_Time < 2250 + 500) // 逻辑1 (2.25ms)
            {
                IR_DATA[IR_pData / 8] |= (0x01 << (IR_pData % 8));
                IR_pData++;
            }
            else 
            {
                IR_pData = 0;
                IR_State = 1;
            }
            
            if (IR_pData >= 32)   // 接收满32位数据
            {
                IR_pData = 0;
                Addr_Temp = ~IR_DATA[1];
                Cmd_Temp  = ~IR_DATA[3];
                if (IR_DATA[0] == Addr_Temp && IR_DATA[2] == Cmd_Temp)  // 数据校验
                {
                    IR_Address = IR_DATA[0];
                    IR_Command = IR_DATA[2];
                    IR_DataFlag = 1;
                }
                IR_State = 0;
            }
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
    }
}
