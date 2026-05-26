#include "chassis.h"
#include "main.h"
#include "can.h"
#include "debug_assistant.h"
#include "XHU_RRC_LIB.h"
#include "can_receive.h"
#include "module_Interface_fops_set.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>

#undef  CHASSIS_K1
#undef  CHASSIS_K2
#undef  CHASSIS_L
#undef  CHASSIS_RECP_L
#undef  CHASSIS_V_X_MIN
#undef  CHASSIS_V_X_MAX
#undef  CHASSIS_V_Y_MIN
#undef  CHASSIS_V_Y_MAX
#undef  CHASSIS_OMEGA_Z_MIN
#undef  CHASSIS_OMEGA_Z_MAX

/* -------------------------------------------- */
#define CHASSIS_USE_OUTER               0
#define Outer_Alpha                     0.9f
/* -------------------------------------------- */
#define CHASSIS_V_X_MIN       (-(__CHASSIS_X_VELOCITY_ABS_MAX__))
#define CHASSIS_V_X_MAX       (__CHASSIS_X_VELOCITY_ABS_MAX__)
#define CHASSIS_V_Y_MIN       (-(__CHASSIS_Y_VELOCITY_ABS_MAX__))
#define CHASSIS_V_Y_MAX       (__CHASSIS_Y_VELOCITY_ABS_MAX__)
#define CHASSIS_OMEGA_Z_MIN   (-(__CHASSIS_Z_PALSTANCE_ABS_MAX__))
#define CHASSIS_OMEGA_Z_MAX   (__CHASSIS_Z_PALSTANCE_ABS_MAX__)
/* -------------------------------------------- */

#define CHASSIS_K1         11.223917f  // K1 = 2^(-1/2)*(1/r) r是轮半径.逆运动学系数.
#define CHASSIS_K2         0.0221857f  // K2 = 2^(1/2)*r/4 r是轮半径.正运动学系数.
#define CHASSIS_L          0.56922096f  // L 是正方形边长.
#define CHASSIS_RECP_L     1.756787034f  // 1/L 边长的倒数.

static PID_t PID_I[4] = {NULL};
#if CHASSIS_USE_OUTER
static PID_t PID_O[3] = {NULL};
#endif
static Motor_t M3508[4] = {NULL};
static PV_t PV[4] = {0};
static float v_end_sp[3] = {0};
static float v_end_pv[3] = {0};
#if CHASSIS_USE_OUTER
static float Outer_CO[3] = {0};
static float Output_O[3] = {0};
#endif
static float q_dot_sp[4] = {0};
static float q_dot_pv[4] = {0};
static int   Inner_CO[4] = {0};

static volatile uint8_t Chassis_direction_mode = 0;

static void Chassis_Set_SP_v_end(float v_x, float v_y, float omega_z);
static void Chassis_Updata_PV(float *pPV, unsigned char Size);

const Chassis_ItfTypeDef Chassis_Interface_fops = {
  .Set_SP = Chassis_Set_SP_v_end,
  .Updata_PV = Chassis_Updata_PV
};

static int __ARM_isfinitef(float __x)
{
  return (isfinite(__x));
}

static inline float CHASSIS_CLAMP(float x, float min, float max)
{
  return ((x > max) ? max : ((x < min) ? min : x));
}

static void Chassis_IK(float v_end[], float q_dot[])
{
  if((v_end != NULL) && (q_dot != NULL))
  {
    float tmp = (-CHASSIS_L * v_end[2]);
    q_dot[0] = CHASSIS_K1 * (  v_end[0] - v_end[1] + tmp );
    q_dot[1] = CHASSIS_K1 * (  v_end[0] + v_end[1] + tmp );
    q_dot[2] = CHASSIS_K1 * ( -v_end[0] + v_end[1] + tmp );
    q_dot[3] = CHASSIS_K1 * ( -v_end[0] - v_end[1] + tmp );
  }
}

static inline void Chassis_Run_PID_Outer(void)
{
  #if CHASSIS_USE_OUTER
  for(int i=0;i<3;++i)
  {
    PIDf_Control(v_end_sp[i],v_end_pv[i],PID_O[i],NULL);
    PID_GetOutput_f(PID_O[i],&Outer_CO[i]);
  }
  for(int i=0;i<3;++i)
  {
    Output_O[i] = (((1.0f - Outer_Alpha) * Outer_CO[i]) + (Outer_Alpha * v_end_sp[i]));
  }
  Chassis_IK(Output_O,q_dot_sp);
  #else
  Chassis_IK(v_end_sp,q_dot_sp);
  #endif
}

static inline void Chassis_Run_PID_Inner(void)
{
  for(int i=0;i<4;++i)
  {
    PIDf_Control(q_dot_sp[i],q_dot_pv[i],PID_I[i],&Inner_CO[i]);
  }
  Chassis_Send_CAN_Cmd(Inner_CO[0],Inner_CO[1],Inner_CO[2],Inner_CO[3]);
}

static void Chassis_Set_PID_Param(void)
{
  PID_Basic_Param_t Param_I = {0};
  #if CHASSIS_USE_OUTER
  PID_Basic_Param_t Param_O = {0};
  Param_O.Kp = 3;
  Param_O.Kd = 0.2f;
  Param_O.Epsilon = 0.001f;
  Param_O.Outputmax = CHASSIS_V_X_MAX;
  PID_SetBasicParam(PID_O[0],&Param_O);
  Param_O.Outputmax = CHASSIS_V_Y_MAX;
  PID_SetBasicParam(PID_O[1],&Param_O);
  Param_O.Epsilon = 0.01f;
  Param_O.Outputmax = CHASSIS_OMEGA_Z_MAX;
  PID_SetBasicParam(PID_O[2],&Param_O);
  #endif
  Param_I.Alpha = 0.8;
  Param_I.Kp = 1150;
  Param_I.Ki = 20;
  Param_I.Kd = 300;
  Param_I.Outputmax = 16384;
  Param_I.Summax = 850;
  Param_I.SumOp_region = 20;
  Param_I.Epsilon = 0;
  for(int i=0;i<4;i++)
  {
    PID_SetBasicParam(PID_I[i],&Param_I);
  }
}

static void Chassis_Set_SP_v_end(float v_x, float v_y, float omega_z)
{
  if(__ARM_isfinitef(v_x)&&__ARM_isfinitef(v_y)&&__ARM_isfinitef(omega_z))
  {
    v_end_sp[0] = CHASSIS_CLAMP(v_x,CHASSIS_V_X_MIN,CHASSIS_V_X_MAX);
    v_end_sp[1] = CHASSIS_CLAMP(v_y,CHASSIS_V_Y_MIN,CHASSIS_V_Y_MAX);
    v_end_sp[2] = CHASSIS_CLAMP(omega_z,CHASSIS_OMEGA_Z_MIN,CHASSIS_OMEGA_Z_MAX);
  }
}

static void Chassis_Update_PV_v_end(float v_x, float v_y, float omega_z)
{
  if(__ARM_isfinitef(v_x)&&__ARM_isfinitef(v_y)&&__ARM_isfinitef(omega_z))
  {
    v_end_pv[0] = v_x;
    v_end_pv[1] = v_y;
    v_end_pv[2] = omega_z;
  }
}

static void Chassis_Update_PV_q_dot(float motor1, float motor2, float motor3, float motor4)
{
  if(__ARM_isfinitef(motor1)&&__ARM_isfinitef(motor2)
        &&__ARM_isfinitef(motor3)&&__ARM_isfinitef(motor4))
  {
    q_dot_pv[0] = motor1;
    q_dot_pv[1] = motor2;
    q_dot_pv[2] = motor3;
    q_dot_pv[3] = motor4;
  }
}

static void Chassis_Updata_PV(float *pPV, unsigned char Size)
{
  if(pPV != NULL)
  {
    switch(Size)
    {
      case 3 : Chassis_Update_PV_v_end(pPV[0],pPV[1],pPV[2]); break;
      case 4 : Chassis_Update_PV_q_dot(pPV[0],pPV[1],pPV[2],pPV[3]); break;
      default : break;
    }
  }
}

uint8_t Chassis_GetDirMode(void)
{
  return Chassis_direction_mode;
}

void Chassis_SetDirMode(uint8_t Mode_Num)
{
  switch (Mode_Num)
  {
    case 0:
    case 1:
    case 2:
    case 3: Chassis_direction_mode = Mode_Num; break;
    default: break;
  }
}

void Chassis_Init(void)
{
  for(int i=0;i<4;i++)
  {
    if(PID_I[i] == NULL)
    {
      PID_I[i] = New_PID_Instance();
      if(PID_I[i] == NULL)
      {
        user_error_handler(__ERR_CHASSIS_INNER_PID);
      }
    }
    if(M3508[i] == NULL)
    {
      M3508[i] = New_M3508_Instance(i,can_bus_1,500);
    }
    #if CHASSIS_USE_OUTER
    if(i<3)
    {
      if(PID_O[i] == NULL)
      {
        PID_O[i] = New_PID_Instance();
        if(PID_O[i] == NULL)
        {
          user_error_handler(__ERR_CHASSIS_OUTER_PID);
        }
      }
    }
    #endif
  }
  Chassis_Set_PID_Param();
}

void Chassis_TIM_PeriodCallback(void)
{
  static uint16_t Counter = 0;
  Counter %= 8;
  for(int i=0;i<4;++i)
  { djiMotor_GetPV(M3508[i],&PV[i]);  }
  Chassis_Update_PV_q_dot(PV[0].Omega,
      PV[1].Omega,PV[2].Omega,PV[3].Omega);
  if(!Counter)
  {
    Chassis_Run_PID_Outer();
  }
  Chassis_Run_PID_Inner();
  Counter++;
}

#undef  CHASSIS_K1
#undef  CHASSIS_K2
#undef  CHASSIS_L
#undef  CHASSIS_RECP_L
#undef  CHASSIS_V_X_MIN
#undef  CHASSIS_V_X_MAX
#undef  CHASSIS_V_Y_MIN
#undef  CHASSIS_V_Y_MAX
#undef  CHASSIS_OMEGA_Z_MIN
#undef  CHASSIS_OMEGA_Z_MAX
