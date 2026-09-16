#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* 函数声明 */
void Motor_Init(void);
//void Motor1_test(void);

void Car_Forward(uint16_t Speed);
void Car_Backward(uint16_t Speed);
void Car_TurnLeft(uint16_t Speed);
void Car_TurnRight(uint16_t Speed);
void Car_TransLeft(uint16_t Speed);
void Car_TransRight(uint16_t Speed);
void Car_Stop(void);

void Motor1_SetSpeed(uint8_t Dir, uint16_t Speed);
void Motor2_SetSpeed(uint8_t Dir, uint16_t Speed);
void Motor3_SetSpeed(uint8_t Dir, uint16_t Speed);
void Motor4_SetSpeed(uint8_t Dir, uint16_t Speed);

void Mechanics_Move(int16_t Vx, int16_t Vy, int16_t W);

#endif /* __MOTOR_H */
