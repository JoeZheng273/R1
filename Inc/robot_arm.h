/**
  ******************************************************************************
  * @file           : robot_arm.h
  * @brief          : 机器人拾取装置
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H

#include "stdint.h"

void RobotArm_Init(void);

void RobotArm_ARM_TriggerFSM(void);

void RobotArm_Arm_Cylinder_Out(void);

void RobotArm_Arm_Cylinder_In(void);

void RobotArm_Arm_Cylinder_Change(void);

void RobotArm_Arm_Force_Cylinder_Change(void);

void RobotArm_Clamp_Change(void);

void RobotArm_Clamp_Disable(void);

void RobotArm_Clamp_Forward(void);

void RobotArm_Clamp_Back(void);

void RobotArm_Clamp_Stop(void);

void RobotArm_Clamp_Enable(void);

_Bool RobotArm_Arm_Processed(uint8_t *pData);

_Bool RobotArm_Clamp_Processed(uint8_t *pData);

_Bool ArmStatus_is_Place(void);

#endif
