#include "remote_ctrl.h"
#include "usart.h"
#include "crc.h"
#include "module_Interface_fops_set.h"
#include "debug_assistant.h"
#include "Critical_Section.h"
#include "user_keymapping.h"
#include "edge_comp.h"
#include "gpio.h"
#include <stdint.h>
#include "chassis.h"

#undef RC_KEY_OP_MASK
#undef RC_KEY_SUPER
#undef RC_FSM_UNINIT
#undef RC_FSM_READY
#undef RC_FSM_HEAD
#undef RC_FSM_NORMAL
#undef RC_FSM_CRCRECEIVE
#undef RC_FSM_TAIL
#undef RC_FSM_ABNORMAL

#define RC_KEY_OP_MASK                0xF0
#define RC_KEY_SUPER                  0x04

#define RC_FSM_UNINIT                 0
#define RC_FSM_READY                  1
#define RC_FSM_HEAD                   2
#define RC_FSM_NORMAL                 3
#define RC_FSM_CRCRECEIVE             4
#define RC_FSM_TAIL                   5
#define RC_FSM_ABNORMAL               6

static uint8_t RC_RxData = 0;  //数据接收缓冲区
static volatile uint8_t RC_STATE = RC_FSM_UNINIT; //状态
static volatile _Bool RC_RxCplt_Flag = 0; //接收完成标志位
static volatile _Bool RC_KeyUpdate_Flag = 0; //按键更新标志位
static uint8_t RC_RxBuffer[4][4] = {0}; //数据接收存储区
static uint8_t RxIndex = 0; //数据接收缓冲区index
static uint8_t DataIndex = 0; //数据index
static volatile uint8_t RC_Counter = 0;  //计数
static volatile uint8_t ReceiveNum = 0; //总计数
static uint32_t RC_CRCinput = 0; //CRC计算输入
static uint32_t RC_CRCreceive = 0;  //CRC接收
static volatile uint8_t KeyValue = 0;
static uint8_t Frame_ID = 0;
static volatile _Bool NotTail = 0;

/* ----- 有限状态机 ----- */
static void RC_FSM(void)
{
  switch(RC_STATE)
  {
    case RC_FSM_READY         : {
      if(RC_RxData == 0xFF)
      {
        RC_STATE = RC_FSM_HEAD;
      }
      else
      {
        RC_STATE = RC_FSM_ABNORMAL;
      }
      break;
    }
    case RC_FSM_HEAD          : {
      if((RC_RxData == 0x0F) || (RC_RxData == 0xF0))
      {
        RC_Counter = 0;
        RxIndex %= 4;
        RC_CRCinput = 0;
        Frame_ID = RC_RxData;
        RC_STATE = RC_FSM_NORMAL;
      }
      else
      {
        RC_STATE = RC_FSM_ABNORMAL;
      }
      break;
    }
    case RC_FSM_NORMAL        : {
      if((RC_Counter >= 0) && (RC_Counter < 4))
      {
        if(Frame_ID == 0x0F)
        {
          RC_RxBuffer[RxIndex][RC_Counter] = RC_RxData;
        }
        else
        {
          RC_RxBuffer[RxIndex][RC_Counter] = 0;
        }
        RC_CRCinput |= (RC_RxData << (24 - (RC_Counter * 8)));
        RC_Counter++;
        if(RC_Counter == 4)
        {
          RxIndex++;
          RC_Counter = 0;
          RC_CRCreceive = 0;
          RC_STATE = RC_FSM_CRCRECEIVE;
        }
      }
      else
      {
        RC_STATE = RC_FSM_ABNORMAL;
      }
      break;
    }
    case RC_FSM_CRCRECEIVE    : {
      if((RC_Counter >= 0) && (RC_Counter < 4))
      {
        RC_CRCreceive |= (RC_RxData <<(RC_Counter * 8));
        RC_Counter++;
        if(RC_Counter == 4)
        {
          if(Frame_ID == 0x0F)
          {
            if(HAL_CRC_Calculate(&hcrc,&RC_CRCinput,1) == RC_CRCreceive)
            {
              RC_STATE = RC_FSM_TAIL;
            }
            else
            {
              RxIndex = (RxIndex + 3)%4;
              for(int i=0;i<4;i++)
              {
                RC_RxBuffer[RxIndex][i] = 0;
              }
              DataIndex = (RxIndex + 1)%4;
              RC_RxCplt_Flag = 1;
              NotTail = 1;
              RC_STATE = RC_FSM_ABNORMAL;
            }
          }
          else if(Frame_ID == 0xF0)
          {
            RC_STATE = RC_FSM_TAIL;
          }
          else
          {
            RC_STATE = RC_FSM_ABNORMAL;
          }
        }
      }
      else
      {
        RC_STATE = RC_FSM_ABNORMAL;
      }
      break;
    }
    case RC_FSM_TAIL          : {
      if(RC_RxData == 0xFE)
      {
        DataIndex = RxIndex;
        RC_RxCplt_Flag = 1;
        RC_STATE = RC_FSM_READY;
      }
      else
      {
        RxIndex = (RxIndex + 3)%4;
        for(int i=0;i<4;i++)
        {
          RC_RxBuffer[RxIndex][i] = 0;
        }
        DataIndex = (RxIndex + 1)%4;
        RC_RxCplt_Flag = 1;
        NotTail = 0;
        RC_STATE = RC_FSM_ABNORMAL;
      }
      break;
    }
    case RC_FSM_ABNORMAL      : {
      if(ReceiveNum == 10)
      {
        if(NotTail)
        {
          NotTail = 0;
        }
        else
        {
          RC_RxCplt_Flag = 0;
          RC_STATE = RC_FSM_READY;
        }
      }
      else if(ReceiveNum <= 2)
      {
        if((DataIndex % 4) == (RxIndex % 4))
        {
          for(int i=0;i<4;i++)
          {
            RC_RxBuffer[RxIndex][i] = 0;
          }
          DataIndex = (RxIndex + 1)%4;
          RC_RxCplt_Flag = 1;
          NotTail = 1;
        }
      }
      else {
      
      }
      break;
    }
    default : break;
  }
}

/* ----------------------------------------------- */
void RC_TurnOn(void)
{
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
}

void RC_TurnOff(void)
{
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
}

void RC_Init(void)
{
  uint8_t tmp = 0x01;
  HAL_UART_Receive_IT(&huart7,&RC_RxData,1);
  HAL_UART_Transmit(&huart7,&tmp,1,5);
  RC_STATE = RC_FSM_READY;
}

void RC_ProcessedData(void)
{
  static uint8_t last_val = 0;
  uint8_t tIndex = (DataIndex + 3)%4;
  int8_t tmp_x = (int8_t)RC_RxBuffer[tIndex][0];
  int8_t tmp_y = (int8_t)RC_RxBuffer[tIndex][1];
  int8_t tmp_z = (int8_t)RC_RxBuffer[tIndex][2];
  uint8_t tmp_val = RC_RxBuffer[tIndex][3];
  RC_RxCplt_Flag = 0;
  if((~RC_KEY_OP_MASK) & tmp_val)
  {
    KeyValue = (((~(RC_KEY_OP_MASK | RC_KEY_SUPER)) & tmp_val) | 
    ((RC_KEY_OP_MASK | RC_KEY_SUPER) & ((tmp_val ^ last_val) & tmp_val)));
    last_val = tmp_val;
  }
  else
  {
    KeyValue = tmp_val;
    last_val = tmp_val;
  }
  RC_KeyUpdate_Flag = 1;
  #if USE_EC
  if(EC_GetState() == 0x0F)
  {
  #endif
    float x = 0.01f * ((float)tmp_x * (float)__CHASSIS_X_VELOCITY_ABS_MAX__) * 0.9f;
    float y = 0.01f * ((float)tmp_y * (float)__CHASSIS_Y_VELOCITY_ABS_MAX__) * 0.9f;
    float z = 0.01f * ((float)tmp_z * (float)__CHASSIS_Z_PALSTANCE_ABS_MAX__) * 0.9f;
    Critical_Enter();
    switch (Chassis_GetDirMode())
    {
      case 0: Chassis_Interface_fops.Set_SP(x,y,z);
      break;
      case 1: Chassis_Interface_fops.Set_SP(-y,x,z);
      break;
      case 2: Chassis_Interface_fops.Set_SP(-x,-y,z);
      break;
      case 3: Chassis_Interface_fops.Set_SP(y,-x,z);
      break;
      default: break;
    }
    Critical_Exit();
  #if USE_EC
  }
  #endif
}

void RC_KeyProcessed(void)
{
  uint8_t tmp = 0;
  RC_KeyUpdate_Flag = 0;
  if(KeyMap_KeyOP_Flag_Reset()) // Reset success return 0.
  {
    user_error_handler(__ERR_RC_KEY_FLAG_RESET);
  }
  else
  {
    //  检查按钮的状态.
    for(int i=0;(i<8)&&KeyValue;i++)
    {
      if(KeyValue & (1 << i)) // 按下.
      {
        switch(i)
        {
          case 0 :
            tmp = 1;  // A
            break;
          case 1 :
            tmp = 2;  // B
            break;
          case 2 :    // Super
            if(KeyMap_Interface_fops.KeyOP_Super != NULL)
            {
              KeyMap_Interface_fops.KeyOP_Super();
            }
            break;
          case 3 :
            tmp = 3;  // C
            break;
          case 4 : {
            switch(tmp)
            {
              case 0 :
                if(KeyMap_Interface_fops.KeyOP_DUp_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_DUp_T();
                }
              break;
              case 1 :
                if(KeyMap_Interface_fops.KeyOP_AUp_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_AUp_T();
                }
              break;
              case 2 :
                if(KeyMap_Interface_fops.KeyOP_BUp_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_BUp_T();
                }
              break;
              case 3 :
                if(KeyMap_Interface_fops.KeyOP_CUp_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_CUp_T();
                }
              break;
              default : break;
            }
          }
          break;
          case 5 : {
            switch(tmp)
            {
              case 0 :
                if(KeyMap_Interface_fops.KeyOP_DLf_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_DLf_T();
                }
              break;
              case 1 :
                if(KeyMap_Interface_fops.KeyOP_ALf_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_ALf_T();
                }
              break;
              case 2 :
                if(KeyMap_Interface_fops.KeyOP_BLf_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_BLf_T();
                }
              break;
              case 3 :
                if(KeyMap_Interface_fops.KeyOP_CLf_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_CLf_T();
                }
              break;
              default : break;
            }
          }
          break;
          case 6 : {
            switch(tmp)
            {
              case 0 :
                if(KeyMap_Interface_fops.KeyOP_DDn_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_DDn_T();
                }
              break;
              case 1 :
                if(KeyMap_Interface_fops.KeyOP_ADn_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_ADn_T();
                }
              break;
              case 2 :
                if(KeyMap_Interface_fops.KeyOP_BDn_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_BDn_T();
                }
              break;
              case 3 :
                if(KeyMap_Interface_fops.KeyOP_CDn_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_CDn_T();
                }
              break;
              default : break;
            }
          }
          break;
          case 7 : {
            switch(tmp)
            {
              case 0 :
                if(KeyMap_Interface_fops.KeyOP_DRt_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_DRt_T();
                }
              break;
              case 1 :
                if(KeyMap_Interface_fops.KeyOP_ARt_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_ARt_T();
                }
              break;
              case 2 :
                if(KeyMap_Interface_fops.KeyOP_BRt_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_BRt_T();
                }
              break;
              case 3 :
                if(KeyMap_Interface_fops.KeyOP_CRt_T != NULL)
                {
                  KeyMap_Interface_fops.KeyOP_CRt_T();
                }
              break;
              default : break;
            }
          }
          break;
          default : break;
        }
      }
      else // 未按下.
      {
        switch(i)
        {
          case 4 : 
            if(KeyMap_Interface_fops.KeyOP_Up_N != NULL)
            {
              KeyMap_Interface_fops.KeyOP_Up_N();
            }
          break;
          case 5 : 
            if(KeyMap_Interface_fops.KeyOP_Lf_N != NULL)
            {
              KeyMap_Interface_fops.KeyOP_Lf_N();
            }
          break;
          case 6 : 
            if(KeyMap_Interface_fops.KeyOP_Dn_N != NULL)
            {
              KeyMap_Interface_fops.KeyOP_Dn_N();
            }
          break;
          case 7 : 
            if(KeyMap_Interface_fops.KeyOP_Rt_N != NULL)
            {
              KeyMap_Interface_fops.KeyOP_Rt_N();
            }
          break;
          default : break;
        }
      }
      KeyValue &= (~(1 << i));
    }
    //  执行按钮相关操作.
    KeyMap_KeyOP_Flag_Processed();
  }
}

_Bool RC_GetRxCpltFlag(void)
{
  return RC_RxCplt_Flag;
}

_Bool RC_GetKeyUpdateFlag(void)
{
  return RC_KeyUpdate_Flag;
}

_Bool RC_SetRxCpltFlag(void)
{
  RC_RxCplt_Flag = 1;
  return RC_RxCplt_Flag;
}

_Bool RC_ResetRxCpltFlag(void)
{
  RC_RxCplt_Flag = 0;
  return RC_RxCplt_Flag;
}

/**
  *@brief   HAL Callback
  **/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == UART7)
  {
    ReceiveNum %= 11;
    HAL_UART_Receive_IT(&huart7,&RC_RxData,1);
    RC_FSM(); // 调用状态机.
    ReceiveNum++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint8_t tmp = (uint8_t)(HAL_UART_GetError(huart) | 0xC0);
  if(huart->Instance == UART7)
  {
    __HAL_UART_CLEAR_NEFLAG(&huart7);
    HAL_UART_RxCpltCallback(&huart7);
  }
  user_debug_test_handler(tmp);
}

#undef RC_KEY_OP_MASK
#undef RC_KEY_SUPER
#undef RC_FSM_UNINIT
#undef RC_FSM_READY
#undef RC_FSM_HEAD
#undef RC_FSM_NORMAL
#undef RC_FSM_CRCRECEIVE
#undef RC_FSM_TAIL
#undef RC_FSM_ABNORMAL
