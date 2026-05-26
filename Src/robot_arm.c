#include "robot_arm.h"
#include "platform.h"
#include "stddef.h"
#include "gpio.h"
#include "can_receive.h"

#undef  Clamp_ID
#undef  Arm_ID
#undef  ARM_FSM_UNINIT
#undef  ARM_FSM_PUSH
#undef  ARM_FSM_OUT_DWELL
#undef  ARM_FSM_RETURN
#undef  ARM_FSM_IN_DWELL
#undef  ARM_FSM_DROP
#undef  ARM_FSM_PLACE
#undef  ARM_FSM_RESET
#undef  CLAMP_FSM_UNINIT
#undef  CLAMP_FSM_REMOTE
#undef  CLAMP_FSM_AUTO

#define Clamp_ID        0x111
#define Arm_ID          0x112

#define ARM_FSM_UNINIT            0
#define ARM_FSM_PUSH              1
#define ARM_FSM_OUT_DWELL         2
#define ARM_FSM_RETURN            3
#define ARM_FSM_IN_DWELL          4
#define ARM_FSM_DROP              5
#define ARM_FSM_PLACE             6
#define ARM_FSM_RESET             7

#define CLAMP_FSM_UNINIT          0
#define CLAMP_FSM_REMOTE          1
#define CLAMP_FSM_AUTO            2

/* -------------------------------------- */
static const uint8_t RobotArm_CMD_Forward[8] = {
  0x02,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xEE
};
static const uint8_t RobotArm_CMD_Back[8] = {
  0x02,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0xEE
};
static const uint8_t RobotArm_CMD_STOP[8] = {
  0x02,0x55,0x55,0x55,0x55,0x55,0x55,0xEE
};

/* -------------------------------------- */
static const uint8_t RobotArm_CMD_ENABLE[8] = {
  0x01,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xEE
};
static const uint8_t RobotArm_CMD_DISABLE[8] = {
  0x01,0xDE,0xDE,0xDE,0xDE,0xDE,0xDE,0xEE
};

/* -------------------------------------- */
static const uint8_t RobotArm_CMD_Ready[8] = {
  0xFF,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xEE
};

/* -------------------------------------- */
static const uint8_t RobotArm_CMD_Push[8] = {
  0x03,0x11,0x11,0x11,0x11,0x11,0x11,0xEE
};
static const uint8_t RobotArm_CMD_Return[8] = {
  0x03,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0xEE
};
static const uint8_t RobotArm_CMD_DROP[8] = {
  0x03,0x15,0x15,0x15,0x15,0x15,0x15,0xEE
};
static const uint8_t RobotArm_CMD_RESET[8] = {
  0x03,0x18,0x18,0x18,0x18,0x18,0x18,0xEE
};

/* -------------------------------------- */
static const uint8_t RobotArm_CMD_CPLT[8] = {
  0x2A,0xFA,0xFA,0xFA,0xFA,0xFA,0xFA,0xEE
};
static const uint8_t RobotArm_CMD_UP[8] = {
  0x2A,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0xEE
};

/* -------------------------------------- */
static volatile uint8_t ARM_STATE = ARM_FSM_UNINIT;
static volatile uint8_t CLAMP_STATE = CLAMP_FSM_UNINIT;
static volatile _Bool Arm_Cplt_Flag = 0;
static volatile _Bool Cylinder_EN = 0;
static volatile _Bool Cylinder_State = 0;
static volatile _Bool Clamp_Disable_Flag = 0;
static volatile _Bool Clamp_Cplt_Flag = 0;
static volatile _Bool Arm_Have = 0;

static void ARM_FSM(void)
{
  switch(ARM_STATE)
  {
    case ARM_FSM_UNINIT : {
      if(RobotArm_Send_CAN_Cmd(Arm_ID,RobotArm_CMD_Ready))
      {
        Arm_Cplt_Flag = 0;
        Cylinder_EN = 0;
        Arm_Have = 0;
        ARM_STATE = ARM_FSM_RESET;
      }
    }
    break;
    case ARM_FSM_PUSH : {
      if(Arm_Cplt_Flag)
      {
        Arm_Cplt_Flag = 0;
        Cylinder_EN = 1;
        ARM_STATE = ARM_FSM_OUT_DWELL;
      }
    }
    break;
    case ARM_FSM_OUT_DWELL : {
      if(RobotArm_Send_CAN_Cmd(Arm_ID,RobotArm_CMD_Return))
      {
        Arm_Cplt_Flag = 0;
        RobotArm_Arm_Cylinder_Out();
        Cylinder_EN = 0;
        ARM_STATE = ARM_FSM_RETURN;
      }
    }
    break;
    case ARM_FSM_RETURN : {
      if(Arm_Cplt_Flag)
      {
        Arm_Cplt_Flag = 0;
        Arm_Have = 1;
        ARM_STATE = ARM_FSM_IN_DWELL;
      }
    }
    break;
    case ARM_FSM_IN_DWELL : {
      if(!Arm_Have)
      {
        if(RobotArm_Send_CAN_Cmd(Arm_ID,RobotArm_CMD_Push))
        {
          Arm_Cplt_Flag = 0;
          Cylinder_EN = 0;
          ARM_STATE = ARM_FSM_PUSH;
        }
      }
      else
      {
        if(RobotArm_Send_CAN_Cmd(Arm_ID,RobotArm_CMD_DROP))
        {
          Arm_Cplt_Flag = 0;
          Cylinder_EN = 0;
          ARM_STATE = ARM_FSM_DROP;
        }
      }
    }
    break;
    case ARM_FSM_DROP : {
      if(Arm_Cplt_Flag)
      {
        Arm_Cplt_Flag = 0;
        Cylinder_EN = 1;
        RobotArm_Arm_Cylinder_In();
        Cylinder_EN = 0;
        ARM_STATE = ARM_FSM_PLACE;
      }
    }
    break;
    case ARM_FSM_PLACE : {
      if(RobotArm_Send_CAN_Cmd(Arm_ID,RobotArm_CMD_RESET))
      {
        Arm_Cplt_Flag = 0;
        Arm_Have = 0;
        ARM_STATE = ARM_FSM_RESET;
      }
    }
    break;
    case ARM_FSM_RESET : {
      if(Arm_Cplt_Flag)
      {
        Arm_Cplt_Flag = 0;
        Arm_Have = 0;
        Cylinder_EN = 1;
        RobotArm_Arm_Cylinder_In();
        Cylinder_EN = 0;
        ARM_STATE = ARM_FSM_IN_DWELL;
      }
    }
    break;
    default : break;
  }
}

static void CLAMP_FSM(void)
{
  switch(CLAMP_STATE)
  {
    case CLAMP_FSM_UNINIT : {
      if(RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_Ready))
      {
        Clamp_Disable_Flag = 0;
        Clamp_Cplt_Flag = 0;
        CLAMP_STATE = CLAMP_FSM_AUTO;
      }
    }
    break;
    case CLAMP_FSM_REMOTE : {
      if(RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_ENABLE))
      {
        Clamp_Disable_Flag = 0;
        Clamp_Cplt_Flag = 0;
        CLAMP_STATE = CLAMP_FSM_AUTO;
      }
    }
    break;
    case CLAMP_FSM_AUTO : {
      if(Clamp_Cplt_Flag || Clamp_Disable_Flag)
      {
        Clamp_Disable_Flag = 0;
        Clamp_Cplt_Flag = 0;
        CLAMP_STATE = CLAMP_FSM_REMOTE;
      }
    }
    break;
    default : break;
  }
}

void RobotArm_Init(void)
{
  CLAMP_FSM();
  HAL_Delay(5);
  ARM_FSM();
}

void RobotArm_ARM_TriggerFSM(void)
{
  if((ARM_STATE == ARM_FSM_IN_DWELL) || (ARM_STATE == ARM_FSM_OUT_DWELL)
      || (ARM_STATE == ARM_FSM_PLACE))
  {
    ARM_FSM();
  }
}

// void RobotArm_Arm_Push(void)
// {
//   if(ARM_STATE == ARM_FSM_IN_DWELL)
//   {
//     ARM_FSM();
//   }
// }

// void RobotArm_Arm_Return(void)
// {
//   if(ARM_STATE == ARM_FSM_OUT_DWELL)
//   {
//     ARM_FSM();
//   }
// }

// void RobotArm_Arm_Drop(void)
// {
//   if(ARM_STATE == ARM_FSM_IN_DWELL)
//   {
//     ARM_FSM();
//   }
// }

// void RobotArm_Arm_Reset(void)
// {
//   if(ARM_STATE == ARM_FSM_PLACE)
//   {
//     ARM_FSM();
//   }
// }

void RobotArm_Arm_Cylinder_Change(void)
{
  if(Cylinder_EN)
  {
    if(Cylinder_State)
    {
      RobotArm_Arm_Cylinder_In();
    }
    else {
      RobotArm_Arm_Cylinder_Out();
    }
  }
}

void RobotArm_Clamp_Change(void)
{
  if(CLAMP_STATE == CLAMP_FSM_AUTO)
  {
    RobotArm_Clamp_Disable();
  }
  else if (CLAMP_STATE == CLAMP_FSM_REMOTE) {
    RobotArm_Clamp_Enable();
  }
}

void RobotArm_Arm_Cylinder_Out(void)
{
  if(Cylinder_EN)
  {
    if(Cylinder_State == 0)
    {
      //  气缸伸长
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
      Cylinder_State = 1;
    }
  }
}

void RobotArm_Arm_Cylinder_In(void)
{
  if(Cylinder_EN)
  {
    if(Cylinder_State == 1)
    {
      //  气缸收缩
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
      Cylinder_State = 0;
    }
  }
}

void RobotArm_Clamp_Disable(void)
{
  if(CLAMP_STATE == CLAMP_FSM_AUTO)
  {
    if(RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_DISABLE))
    {
      Clamp_Disable_Flag = 1;
      CLAMP_FSM();
    }
  }
}

void RobotArm_Clamp_Forward(void)
{
  if(CLAMP_STATE == CLAMP_FSM_REMOTE)
  {
    RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_Forward);
  }
}

void RobotArm_Clamp_Back(void)
{
  if(CLAMP_STATE == CLAMP_FSM_REMOTE)
  {
    RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_Back);
  }
}

void RobotArm_Clamp_Stop(void)
{
  if(CLAMP_STATE == CLAMP_FSM_REMOTE)
  {
    RobotArm_Send_CAN_Cmd(Clamp_ID,RobotArm_CMD_STOP);
  }
}

void RobotArm_Clamp_Enable(void)
{
  if(CLAMP_STATE == CLAMP_FSM_REMOTE)
  {
    CLAMP_FSM();
  }
}

_Bool RobotArm_Arm_Processed(uint8_t *pData)
{
  _Bool tmp = 0;
  if(pData != NULL)
  {
    if((pData[0] == 0x2A) && (pData[7] == 0xEE))
    {
      if(pData[1] == RobotArm_CMD_CPLT[1])
      {
        tmp = 1;
        Arm_Cplt_Flag = 1;
        ARM_FSM();
      }
      else
      { }
    }
  }
  return tmp;
}

_Bool RobotArm_Clamp_Processed(uint8_t *pData)
{
  _Bool tmp = 0;
  if(pData != NULL)
  {
    if((pData[0] == 0x2A) && (pData[7] == 0xEE))
    {
      if(pData[1] == RobotArm_CMD_CPLT[1])
      {
        tmp = 1;
        Clamp_Cplt_Flag = 1;
        CLAMP_FSM();
      }
      else if (pData[1] == RobotArm_CMD_UP[1])
      {
        tmp = 1;
        PlatForm_High_Add_delta();
      }
      else
      { }
    }
  }
  return tmp;
}

#undef  Clamp_ID
#undef  Arm_ID
#undef  ARM_FSM_UNINIT
#undef  ARM_FSM_PUSH
#undef  ARM_FSM_OUT_DWELL
#undef  ARM_FSM_RETURN
#undef  ARM_FSM_IN_DWELL
#undef  ARM_FSM_DROP
#undef  ARM_FSM_PLACE
#undef  ARM_FSM_RESET
#undef  CLAMP_FSM_UNINIT
#undef  CLAMP_FSM_REMOTE
#undef  CLAMP_FSM_AUTO
