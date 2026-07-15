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
#include "debug_assistant.h"
#include "bsp_led.h"
#include "main.h"

#if Serial_Plot_Enable
#include "usart.h"
static volatile _Bool serial_plot_flag = 0;
#endif

void user_error_handler(unsigned char error_code)
{
  if(error_code)
  {
    uint16_t tmp = error_code << 1;
    if(tmp < 0x1FF)
    {
      bsp_LED_GREEN_OFF();
      bsp_LED_x_ON(tmp);
    }
    else
    {
      bsp_LED_ALL_ON();
    }
    Error_Handler();
  }
}

_Bool user_debug_test_handler(unsigned char test_code)
{
  _Bool tbool = 0;
  if(test_code)
  {
    uint16_t tmp = test_code << 1;
    if(tmp < 0x1FF)
    {
      bsp_LED_GREEN_ON();
      bsp_LED_RED_OFF();
      bsp_LED_x_OFF(0x1FE);
      bsp_LED_x_ON(tmp);
      tbool = 1;
    }
    else
    {
      bsp_LED_ALL_ON();
    }
  }
  else
  {
    bsp_LED_x_OFF(0x1FE);
    tbool = 1;
  }
  return tbool;
}

#if Serial_Plot_Enable

static inline void usart_senddatas(UART_HandleTypeDef* USARTx,uint8_t* addr,int size,uint32_t TimeOut)
{
  HAL_UART_Transmit(USARTx,addr,size,TimeOut);
}

_Bool serial_plot_flag_set(void)
{
  serial_plot_flag = 1;
  return serial_plot_flag;
}

_Bool serial_plot_flag_reset(void)
{
  serial_plot_flag = 0;
  return serial_plot_flag;
}

_Bool serial_plot_get_flag(void)
{
  return serial_plot_flag;
}

//相关数据发送生成波形。
void serial_plot_send(const Serial_Plot_Packet_t *pPacket)
{
  //定义通道名帧头帧尾
  uint8_t frameNameHead[] = "AABBCC";
  uint8_t frameNameEnd[] = "CCBBAA";
  
  //定义数据帧头帧尾
  uint8_t frameDataHead[] = "DDEEFF";
  uint8_t frameDataEnd[] = "FFEEDD";
  
  //通过串口1，向上位机发送数据
  //下面sizeof(frameNameHead)-1) 中的减1，就是忽略掉frameNameHead字符串最后的 ‘\0’ 字符
  usart_senddatas(pPacket->huart,frameNameHead,sizeof(frameNameHead)-1,pPacket->timeout_ms);
  usart_senddatas(pPacket->huart,pPacket->name,pPacket->name_len-1,pPacket->timeout_ms);
  usart_senddatas(pPacket->huart,frameNameEnd,sizeof(frameNameEnd)-1,pPacket->timeout_ms);
  usart_senddatas(pPacket->huart,frameDataHead,sizeof(frameDataHead)-1,pPacket->timeout_ms);
  usart_senddatas(pPacket->huart,(uint8_t*)pPacket->plot_data,pPacket->data_len,pPacket->timeout_ms);
  usart_senddatas(pPacket->huart,frameDataEnd,sizeof(frameDataEnd)-1,pPacket->timeout_ms);
}

#endif
