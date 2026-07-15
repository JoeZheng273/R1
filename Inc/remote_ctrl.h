/**
  ******************************************************************************
  * @file           : remote_ctrl.h
  * @brief          : 遥控器数据接收与处理
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __REMOTE_CTRL_H
#define __REMOTE_CTRL_H

#include "stdint.h"

void RC_TurnOn(void);

void RC_TurnOff(void);

void RC_Init(void);

void RC_ProcessedData(void);

void RC_KeyProcessed(void);

_Bool RC_GetKeyUpdateFlag(void);

_Bool RC_GetRxCpltFlag(void);

_Bool RC_SetRxCpltFlag(void);

_Bool RC_ResetRxCpltFlag(void);

#endif
