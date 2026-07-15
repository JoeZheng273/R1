/**
  ******************************************************************************
  * @file           : chassis.h
  * @brief          : 机器人底盘
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stdint.h"

void Chassis_Init(void);

uint8_t Chassis_GetDirMode(void);

void Chassis_SetDirMode(uint8_t Mode_Num);

void Chassis_SetGain(float Gain);

void Chassis_TIM_PeriodCallback(void);

#endif
