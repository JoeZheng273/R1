#include "stdint.h"
#include "stdio.h"
#include "safe_task.h"
#include "platform.h"
#include "robot_arm.h"
#include "edge_comp.h"
#include "chassis.h"
#include "debug_assistant.h"
#include "module_Interface_fops_set.h"

#undef KEYOP_PLAT_UP
#undef KEYOP_PLAT_DN
#undef KEYOP_CLAMP_F
#undef KEYOP_CLAMP_B

#define KEYOP_PLAT_UP           0x01
#define KEYOP_PLAT_DN           0x02
#define KEYOP_CLAMP_F           0x10
#define KEYOP_CLAMP_B           0x20

static volatile uint8_t KeyOP_Flag = 0;

static void KeyMap_KeyOP_Flag_SetPlatUp(void);
static void KeyMap_KeyOP_Flag_SetPlatDn(void);
static void KeyMap_KeyOP_Flag_SetClampForword(void);
static void KeyMap_KeyOP_Flag_SetClampBack(void);
static void KeyMap_Arm_TriggerFSM(void);
static void KeyMap_PlatConv_Forward(void);
static void KeyMap_PlatConv_Back(void);
static void KeyMap_PlatConv_Stop(void);
// static void KeyMap_EC_ChangeState(void);
static void KeyMap_Chassis_SetMode_0(void);
static void KeyMap_Chassis_SetMode_1(void);
static void KeyMap_Chassis_SetMode_2(void);
static void KeyMap_Chassis_SetMode_3(void);
static void KeyMap_Arm_Cylinder_Change(void);
static void KeyMap_Arm_Force_Cylinder_Change(void);
static void KeyMap_Clamp_Change(void);

#if (USER == USER_alpha)
const KeyMapping_ItfTypeDef KeyMap_Interface_fops = {
  .KeyOP_DUp_T = KeyMap_KeyOP_Flag_SetPlatUp        ,
  .KeyOP_DLf_T = KeyMap_KeyOP_Flag_SetClampBack     ,
  .KeyOP_DDn_T = KeyMap_KeyOP_Flag_SetPlatDn        ,
  .KeyOP_DRt_T = KeyMap_KeyOP_Flag_SetClampForword  ,
  .KeyOP_AUp_T = KeyMap_Chassis_SetMode_0           ,
  .KeyOP_ALf_T = KeyMap_Chassis_SetMode_1           ,
  .KeyOP_ADn_T = KeyMap_Chassis_SetMode_2           ,
  .KeyOP_ARt_T = KeyMap_Chassis_SetMode_3           ,
  .KeyOP_BUp_T = NULL                               ,
  .KeyOP_BLf_T = KeyMap_Arm_Cylinder_Change         ,
  .KeyOP_BDn_T = NULL                               ,
  .KeyOP_BRt_T = KeyMap_Arm_TriggerFSM              ,
  .KeyOP_CUp_T = KeyMap_Clamp_Change                ,
  .KeyOP_CLf_T = KeyMap_PlatConv_Back               ,
  .KeyOP_CDn_T = KeyMap_PlatConv_Stop               ,
  .KeyOP_CRt_T = KeyMap_PlatConv_Forward            ,
  .KeyOP_Up_N  = NULL                               ,
  .KeyOP_Lf_N  = NULL                               ,
  .KeyOP_Dn_N  = NULL                               ,
  .KeyOP_Rt_N  = NULL                               ,
  .KeyOP_Super = KeyMap_Arm_Force_Cylinder_Change   ,
};
#elif (USER == USER_1)
const KeyMapping_ItfTypeDef KeyMap_Interface_fops = {
  .KeyOP_DUp_T = NULL,
  .KeyOP_DLf_T = NULL,
  .KeyOP_DDn_T = NULL,
  .KeyOP_DRt_T = NULL,
  .KeyOP_AUp_T = NULL,
  .KeyOP_ALf_T = NULL,
  .KeyOP_ADn_T = NULL,
  .KeyOP_ARt_T = NULL,
  .KeyOP_BUp_T = NULL,
  .KeyOP_BLf_T = NULL,
  .KeyOP_BDn_T = NULL,
  .KeyOP_BRt_T = NULL,
  .KeyOP_CUp_T = NULL,
  .KeyOP_CLf_T = NULL,
  .KeyOP_CDn_T = NULL,
  .KeyOP_CRt_T = NULL,
  .KeyOP_Up_N  = NULL,
  .KeyOP_Lf_N  = NULL,
  .KeyOP_Dn_N  = NULL,
  .KeyOP_Rt_N  = NULL,
  .KeyOP_Super = NULL,
};
#elif (USER == USER_2)
const KeyMapping_ItfTypeDef KeyMap_Interface_fops = {
  .KeyOP_DUp_T = NULL,
  .KeyOP_DLf_T = NULL,
  .KeyOP_DDn_T = NULL,
  .KeyOP_DRt_T = NULL,
  .KeyOP_AUp_T = NULL,
  .KeyOP_ALf_T = NULL,
  .KeyOP_ADn_T = NULL,
  .KeyOP_ARt_T = NULL,
  .KeyOP_BUp_T = NULL,
  .KeyOP_BLf_T = NULL,
  .KeyOP_BDn_T = NULL,
  .KeyOP_BRt_T = NULL,
  .KeyOP_CUp_T = NULL,
  .KeyOP_CLf_T = NULL,
  .KeyOP_CDn_T = NULL,
  .KeyOP_CRt_T = NULL,
  .KeyOP_Up_N  = NULL,
  .KeyOP_Lf_N  = NULL,
  .KeyOP_Dn_N  = NULL,
  .KeyOP_Rt_N  = NULL,
  .KeyOP_Super = NULL,
};
#else
const KeyMapping_ItfTypeDef KeyMap_Interface_fops = {
  .KeyOP_DUp_T = NULL,
  .KeyOP_DLf_T = NULL,
  .KeyOP_DDn_T = NULL,
  .KeyOP_DRt_T = NULL,
  .KeyOP_AUp_T = NULL,
  .KeyOP_ALf_T = NULL,
  .KeyOP_ADn_T = NULL,
  .KeyOP_ARt_T = NULL,
  .KeyOP_BUp_T = NULL,
  .KeyOP_BLf_T = NULL,
  .KeyOP_BDn_T = NULL,
  .KeyOP_BRt_T = NULL,
  .KeyOP_CUp_T = NULL,
  .KeyOP_CLf_T = NULL,
  .KeyOP_CDn_T = NULL,
  .KeyOP_CRt_T = NULL,
  .KeyOP_Up_N  = NULL,
  .KeyOP_Lf_N  = NULL,
  .KeyOP_Dn_N  = NULL,
  .KeyOP_Rt_N  = NULL,
  .KeyOP_Super = NULL,
};
#endif

static void KeyMap_KeyOP_Flag_SetPlatUp(void)
{
  if(!(Safe_task_Get_LimFlag(0) || Safe_task_Get_LimFlag(2)))
  {
    KeyOP_Flag |= KEYOP_PLAT_UP; //  平台抬升
  }
}

static void KeyMap_KeyOP_Flag_SetPlatDn(void)
{
  if(!(Safe_task_Get_LimFlag(1) || Safe_task_Get_LimFlag(3)))
  {
    KeyOP_Flag |= KEYOP_PLAT_DN; //  平台下降
  }
}

static void KeyMap_KeyOP_Flag_SetClampForword(void)
{
  KeyOP_Flag |= KEYOP_CLAMP_F;
}

static void KeyMap_KeyOP_Flag_SetClampBack(void)
{
  KeyOP_Flag |= KEYOP_CLAMP_B;
}

static void KeyMap_Clamp_Change(void)
{
  RobotArm_Clamp_Change();
}

static void KeyMap_Arm_TriggerFSM(void)
{
  RobotArm_ARM_TriggerFSM();
}

static void KeyMap_Arm_Cylinder_Change(void)
{
  RobotArm_Arm_Cylinder_Change();
}

// static void KeyMap_EC_ChangeState(void)
// {
//   EC_ChangeState();
// }

static void KeyMap_Arm_Force_Cylinder_Change(void)
{
  RobotArm_Arm_Force_Cylinder_Change();
}

static void KeyMap_Chassis_SetMode_0(void)
{
  Chassis_SetDirMode(0);
}

static void KeyMap_Chassis_SetMode_1(void)
{
  Chassis_SetDirMode(1);
}

static void KeyMap_Chassis_SetMode_2(void)
{
  Chassis_SetDirMode(2);
}

static void KeyMap_Chassis_SetMode_3(void)
{
  Chassis_SetDirMode(3);
}

static void KeyMap_PlatConv_Forward(void)
{
  PlatForm_Conveyor_Forward();
}

static void KeyMap_PlatConv_Back(void)
{
  PlatForm_Conveyor_Back();
}

static void KeyMap_PlatConv_Stop(void)
{
  PlatForm_Conveyor_Stop();
}

_Bool KeyMap_KeyOP_Flag_Reset(void)
{
  KeyOP_Flag = 0;
  return KeyOP_Flag;
}

void KeyMap_KeyOP_Flag_Processed(void)
{
  switch(KeyOP_Flag & 0x0F)
  {
    case 0x00 :
      PlatForm_High_Stop();
    break;
    case KEYOP_PLAT_UP :
      PlatForm_High_Add();
    break;
    case KEYOP_PLAT_DN :
      PlatForm_High_Sub();
    break;
    default :
      PlatForm_High_Stop();
    break;
  }
  switch(KeyOP_Flag & 0xF0)
  {
    case 0x00 :
      RobotArm_Clamp_Stop();
    break;
    case KEYOP_CLAMP_F :
      RobotArm_Clamp_Forward();
    break;
    case KEYOP_CLAMP_B :
      RobotArm_Clamp_Back();
    break;
    default :
      RobotArm_Clamp_Stop();
    break;
  }
}

#undef KEYOP_PLAT_UP
#undef KEYOP_PLAT_DN
#undef KEYOP_CLAMP_F
#undef KEYOP_CLAMP_B
