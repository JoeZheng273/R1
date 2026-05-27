#include "safe_task.h"
#include "Critical_Section.h"
#include "gpio.h"
#include "tim.h"
#include "platform.h"
#include "XHU_RRC_LIB.h"
#include <stdint.h>

#undef    SAFE_TASK_LIM_KEY_NUM
#define   SAFE_TASK_LIM_KEY_NUM           4

static volatile _Bool LimFlag[SAFE_TASK_LIM_KEY_NUM] = {0}; //限位开关.

static volatile _Bool AutoTask = 0;
static volatile uint8_t AutoTaskCounter = 0;

static _Bool Read_LimKeyPin_x(uint16_t Pin_x)
{
  _Bool tmp = 1;
  switch(Pin_x)
  {
    case 0:
      if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == GPIO_PIN_SET)
      {
        tmp = 0;
      }
      break;
    case 1:
      if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0) == GPIO_PIN_SET)
      {
        tmp = 0;
      }
      break;
    case 2:
      if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6) == GPIO_PIN_SET)
      {
        tmp = 0;
      }
      break;
    case 3:
      if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12) == GPIO_PIN_SET)
      {
        tmp = 0;
      }
      break;
    default : break;
  }
  return tmp;
}

static void Safe_task_AutoTaskUpdate(void)
{
  if(PlatForm_High_TargetExecute_GetFlag())
  {
    PlatForm_High_TargetExecute_ResetFlag();
    AutoTask = 1;
    AutoTaskCounter = 50;
  }
  if(AutoTask)
  {
    if(!AutoTaskCounter)
    {
      AutoTaskCounter = 0;
      AutoTask = 0;
    }
    else {
      AutoTaskCounter--;
    }
  }
}

_Bool Safe_task_GetHighLim(void)
{
  return (LimFlag[0] || LimFlag[2]);
}

_Bool Safe_task_GetLowLim(void)
{
  return (LimFlag[1] || LimFlag[3]);
}

void Safe_task_Update_LimFlag(void)
{
  for(int i=0;i<SAFE_TASK_LIM_KEY_NUM;i++)
  {
    LimFlag[i] = Read_LimKeyPin_x(i);
  }
}

_Bool Safe_task_Get_LimFlag(uint16_t Key_ID)
{
  if(Key_ID < SAFE_TASK_LIM_KEY_NUM)
  {
    return LimFlag[Key_ID];
  }
  else
  {
    return 1;
  }
}

_Bool Safe_task_GetAutoTaskFlag(void)
{
  return AutoTask;
}

_Bool Safe_task_ResetAutoTaskFlag(void)
{
  Critical_Enter();
  AutoTaskCounter = 0;
  AutoTask = 0;
  Critical_Exit();
  return AutoTask;
}

void Safe_task_TIM_PeriodCallback(void)
{
  Safe_task_Update_LimFlag();
  Safe_task_AutoTaskUpdate();
}

void Safe_task_Back_to_zero(void)
{
  //Start Back to zero
  HAL_TIM_Base_Stop_IT(&htim6);
  PlatForm_BackToZero_Start();
  HAL_TIM_Base_Start_IT(&htim6);
  while(!(LimFlag[1] && LimFlag[3]))
  {
    for(int i=0;(i<20)&&(!(LimFlag[1] && LimFlag[3]));i++)
    {
      HAL_Delay(2);
    }
  }
  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_Delay(5);
  //End
  PlatForm_BackToZero_End();
  HAL_TIM_Base_Start_IT(&htim6);
}

#undef    SAFE_TASK_LIM_KEY_NUM
