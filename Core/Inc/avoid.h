#ifndef __AVOID_H
#define __AVOID_H

#include "main.h"

void Avoid_Init(void);
void Avoid_Check_Obstacle(void);
uint8_t Avoid_Get_Emergency_Flag(void);
float Avoid_Get_Distance(void);

#endif
