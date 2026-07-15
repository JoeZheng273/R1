/**
  ******************************************************************************
  * @file           : debug_assistant.h
  * @brief          : 调试助手
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __DEBUG_ASSISTANT_H
#define __DEBUG_ASSISTANT_H

#define Serial_Plot_Enable                  0

#if Serial_Plot_Enable

typedef struct __UART_HandleTypeDef  UART_Handle_t;

typedef struct
{
  UART_Handle_t *huart;
  unsigned char* name;
  unsigned int name_len;
  float *plot_data;
  unsigned int data_len;
  unsigned int timeout_ms;
} Serial_Plot_Packet_t;

_Bool serial_plot_flag_set(void);

_Bool serial_plot_flag_reset(void);

_Bool serial_plot_get_flag(void);

void serial_plot_send(const Serial_Plot_Packet_t *pPacket);

#endif

void user_error_handler(unsigned char error_code);

_Bool user_debug_test_handler(unsigned char test_code);

#endif
