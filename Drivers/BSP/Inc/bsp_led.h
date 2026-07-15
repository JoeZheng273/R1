/**
  ******************************************************************************
  * @file           : bsp_led.h
  * @brief          : LED指示灯驱动
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stdint.h"

#define LED_1                 ((uint16_t)0x0002u)
#define LED_2                 ((uint16_t)0x0004u)
#define LED_3                 ((uint16_t)0x0008u)
#define LED_4                 ((uint16_t)0x0010u)
#define LED_5                 ((uint16_t)0x0020u)
#define LED_6                 ((uint16_t)0x0040u)
#define LED_7                 ((uint16_t)0x0080u)
#define LED_8                 ((uint16_t)0x0100u)

void bsp_LED_Init(void);

void bsp_LED_ALL_ON(void);

void bsp_LED_ALL_OFF(void);

void bsp_LED_x_ON(uint16_t LED_x);

void bsp_LED_x_OFF(uint16_t LED_x);

void bsp_LED_GREEN_ON(void);

void bsp_LED_GREEN_OFF(void);

void bsp_LED_RED_ON(void);

void bsp_LED_RED_OFF(void);

#endif
