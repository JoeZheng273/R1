#include "edge_comp.h"
#include "stdio.h"
#include "crc.h"
#include "debug_assistant.h"
#include "usbd_cdc_if.h"
#include "module_Interface_fops_set.h"
#include "Critical_Section.h"

static const uint8_t EC_CMD_START_DATA[8] = {
  0xFF,0x0F,0x0F,0x01,0x10,0xF0,0xF0,0xFE };
static const uint8_t EC_CMD_AUTO_DATA[8] = {
  0xFF,0x0F,0x0F,0x02,0x20,0xF0,0xF0,0xFE };
static const uint8_t EC_CMD_REMOTE_DATA[8] = {
  0xFF,0x0F,0x0F,0x04,0x40,0xF0,0xF0,0xFE };

static uint32_t RxData[6] = {0};
static volatile _Bool EC_RxCplt_Flag = 0;
static volatile uint32_t _size = 0;
static volatile uint8_t *pBuffer = NULL;
static volatile uint8_t STATE = 0;

extern uint8_t UserRxBufferFS[];
extern uint8_t UserTxBufferFS[];

void EC_Init(void)
{
  STATE = 0x01;
  // user_debug_test_handler(STATE);
}

void EC_Control(EC_CMD CMD)
{
  switch(CMD)
  {
    case EC_CMD_START  : 
      CDC_Transmit_FS((uint8_t*)EC_CMD_START_DATA,8);
      break;
    case EC_CMD_AUTO   : 
      CDC_Transmit_FS((uint8_t*)EC_CMD_AUTO_DATA,8);
      break;
    case EC_CMD_REMOTE : 
      CDC_Transmit_FS((uint8_t*)EC_CMD_REMOTE_DATA,8);
      break;
    default : break;
  }
}

uint8_t EC_GetState(void)
{
  return STATE;
}

void EC_ChangeState(void)
{
  if(STATE == 0x0F)
  {
    EC_Control(EC_CMD_AUTO);
    STATE = 0xF0;
  }
  else
  {
    EC_Control(EC_CMD_REMOTE);
    STATE = 0x0F;
  }
}

_Bool EC_GetRxCpltFlag(void)
{
  return EC_RxCplt_Flag;
}

_Bool EC_SetRxCpltFlag(uint8_t *pRx, uint32_t Size)
{
  _size = Size;
  pBuffer = pRx;
  EC_RxCplt_Flag = 1;
  return EC_RxCplt_Flag;
}

_Bool EC_ResetRxCpltFlag(void)
{
  _size = 0;
  pBuffer = NULL;
  EC_RxCplt_Flag = 0;
  return EC_RxCplt_Flag;
}

void EC_ProcessedData(void)
{
  uint32_t tmp = 0;
  EC_RxCplt_Flag = 0;
  if(pBuffer == UserRxBufferFS)
  {
    if(UserRxBufferFS[0] == 0xFF)
    {
      if(UserRxBufferFS[UserRxBufferFS[2]+7] == 0xFE)
      {
        if(UserRxBufferFS[1] == STATE) // 对帧ID进行判断
        {
          //  以下对USB的帧ID进行分类.
          if(UserRxBufferFS[1] == 0x0F) //  遥控模式
          {
            if(UserRxBufferFS[2] == 0x18)
            {
              for(int i=0;i<6;i++)
              {
                RxData[i] = 0;
              }
              for(int i=0;i<6;i++)
              {
                for(int j=0;j<4;j++)
                {
                  RxData[i] |= (UserRxBufferFS[j+3+(4*i)] << (j * 8));
                }
              }
              for(int i=0;i<4;i++)
              {
                tmp |= (UserRxBufferFS[UserRxBufferFS[2]+3+i] << (i * 8));
              }
              if(HAL_CRC_Calculate(&hcrc,RxData,6) == tmp)
              {
                Critical_Enter();
                float *tmp_rx = (float *)RxData;
                user_debug_test_handler(STATE);
                Chassis_Interface_fops.Updata_PV(tmp_rx,3);
                Critical_Exit();
              }
            }
          }
          else if(UserRxBufferFS[1] == 0xF0) // 自动模式
          {
            user_debug_test_handler(STATE);
          }
          else if(UserRxBufferFS[1] == 0x01) // 初始化模式
          {
            EC_Control(EC_CMD_REMOTE);
            STATE = 0x0F;
            user_debug_test_handler(STATE);
          }
          else
          {  }
        }
        else // ID与状态不相符合.
        {
        }
      }
    }
  }
  _size = 0;
  pBuffer = NULL;
}
