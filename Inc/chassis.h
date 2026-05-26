#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stdint.h"

void Chassis_Init(void);

uint8_t Chassis_GetDirMode(void);

void Chassis_SetDirMode(uint8_t Mode_Num);

void Chassis_TIM_PeriodCallback(void);

#endif
