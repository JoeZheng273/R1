#include "platform.h"
#include "Critical_Section.h"
#include "main.h"
#include "can.h"
#include "arm_math.h"
#include "debug_assistant.h"
#include "XHU_RRC_LIB.h"
#include "can_receive.h"
#include "safe_task.h"

#undef  OUTLOOP_PERIOD
#undef  HIGH_NUM
#undef  HIGH_delta
#undef  Angle_delta
#undef  INNER_Alpha
#undef  ERROR_LIM
#undef  PLATFORM_HIGH_BASE
#undef  PLATFORM_HIGH_MIN
#undef  PLATFORM_HIGH_MAX
#undef  fAlpha
#undef  Conveyor_Alpha

#define OUTLOOP_PERIOD                5
#define HIGH_NUM                      6
#define HIGH_delta                    (0.2139f * 64.0f)              //0.02139f
#define Angle_delta                   (0.672f * 64.0f)
#define INNER_Alpha                   0.5f
#define ERROR_LIM                     0.25f
#define PLATFORM_HIGH_BASE            0.22f                         //  unit:m.
#define PLATFORM_HIGH_MAX             0.59f
#define PLATFORM_HIGH_MIN             PLATFORM_HIGH_BASE
#define fAlpha                        0.7f
#define Conveyor_Alpha                0.5f

static const float HighForm[HIGH_NUM] = {0.2f,0.4f,0.6f,0.6f,0.6f};

static PID_t PID_M[2] = {NULL};
static PID_t PID_S[2] = {NULL};
static PID_t PID_c = NULL;
static Motor_t M3508_M = NULL;
static Motor_t M3508_S = NULL;
static Motor_t M2006 = NULL;
static float SP_M[2] = {0};
static float SP_S[2] = {0};
static float SP_c = 0;
static PV_t PV_M = {0};
static PV_t PV_S = {0};
static PV_t PV_c = {0};
static int CO_M = 0;
static int CO_S = 0;
static int CO_c = 0;
static volatile uint8_t HighCounter = 0;
static volatile float dH = 0;
static volatile float High = 0;
static volatile float HighAngle = 0;
static volatile _Bool TargetExecute = 0;

static inline float PLATFORM_CLAMP(float min, float x, float max)
{
  return ((x > max) ? max : ((x < min) ? min : x));
}

/* -------------------------------------------------- */
static float Forward(const float Target, const PID_Inst_t* const This)
{
  static float SP_last_M = 0;
  static float SP_last_S = 0;
  float tmp = 0.0f;
  if(This == PID_M[1])
  {
    tmp = 500.0f * (Target - SP_last_M) + 60.0f * 
    ((Target > 0.0f)? 1.0f :((Target < -0.0f)? -1.0f : 0.0f));
    SP_last_M = (fAlpha * Target) + ((1-fAlpha) * SP_last_M);
  }
  else if(This == PID_S[1])
  {
    tmp = 500.0f * (Target - SP_last_S) + 60.0f * 
    ((Target > 0.0f)? 1.0f :((Target < -0.0f)? -1.0f : 0.0f));
    SP_last_S = (fAlpha * Target) + ((1-fAlpha) * SP_last_S);
  }
  else
  {
  }
  return tmp;
}

static float Forward_S(const float Target, const PID_Inst_t* const This)
{
  return 0.0f;
}

/* -------------------------------------------------- */
static inline void PlatForm_Set_PID_Param(void)
{
  PID_Basic_Param_t Param = {0};
  PID_Basic_Param_t Param_O = {0};
  /* -------------------------- */
  Param_O.Alpha = 0.8;
  Param_O.Kp = 10;
  Param_O.Ki = 0;
  Param_O.Kd = 5;
  Param_O.Kf = 0;
  Param_O.Outputmax = 35;
  Param_O.Summax = 0;
  Param_O.SumOp_region = 0;
  Param_O.Epsilon = 0.01;
  /* -------------------------- */
  Param.Alpha = 0.8;
  Param.Kp = 1150;
  Param.Ki = 20;
  Param.Kd = 300;
  Param.Kf = 1;
  Param.Outputmax = 16380;
  Param.Summax = 850;
  Param.SumOp_region = 20;
  Param.Epsilon = 0;
  /* -------------------------- */
  PID_SetBasicParam(PID_c,&Param);
  PID_SetBasicParam(PID_M[1],&Param);
  PID_SetParam_Forward(PID_M[1],Forward);
  PID_SetBasicParam(PID_S[1],&Param);
  PID_SetParam_Forward(PID_S[1],Forward);
  /* -------------------------- */
  PID_SetBasicParam(PID_M[0],&Param_O);
  Param_O.Kf = 1;
  PID_SetBasicParam(PID_S[0],&Param_O);
  PID_SetParam_Forward(PID_S[0],Forward_S);
}

/* -------------------------------------------------- */
static float PlatForm_IK(float deltaHigh) // unit: m.
{
//  (deltaHigh / 0.006f) * (2.0f * PI)
  if(fabsf(deltaHigh) < 1.0E-5)
  {
    deltaHigh = 0.0f;
  }
  return (1047.1975511966f * deltaHigh);
}

static float PlatForm_FK(float Angle)
{
//  (Angle / (2.0f * PI)) * 0.006f
  if(fabsf(Angle) < 1.0E-2)
  {
    Angle = 0.0f;
  }
  return (0.00095492965855f * Angle);
}

static float AngleUpdate(float NowAngle, float TargetAngle)
{
  if(fabsf(NowAngle-TargetAngle) > Angle_delta)
  {
    if(NowAngle > TargetAngle)
    {
      return (NowAngle - Angle_delta);
    }
    else
    {
      return (NowAngle + Angle_delta);
    }
  }
  else
  {
    return TargetAngle;
  }
}

/* -------------------------------------------------- */
void PlatForm_Init(void)
{
  PID_M[0] = New_PID_Instance();
  PID_M[1] = New_PID_Instance();
  PID_S[0] = New_PID_Instance();
  PID_S[1] = New_PID_Instance();
  PID_c = New_PID_Instance();
  /* -------------------------------------------- */
  M2006 = New_M2006_Instance(ID_5,can_bus_1,500);
  M3508_M = New_M3508_Instance(ID_6,can_bus_1,500);
  M3508_S = New_M3508_Instance(ID_7,can_bus_1,500);
  /* -------------------------------------------- */
  PlatForm_Set_PID_Param();
}

void PlatForm_BackToZero_Start(void)
{
  High = PLATFORM_HIGH_MIN;
  djiMotor_Set_round(M3508_M,300000);
  djiMotor_Set_round(M3508_S,300000);
}

void PlatForm_BackToZero_End(void)
{
  High = PLATFORM_HIGH_MIN;
  djiMotor_Back2Zero(M3508_M);
  djiMotor_Back2Zero(M3508_S);
  HighAngle = 0;
  SP_M[0] = 0;
}

void PlatForm_High_Up(void)
{
  Critical_Enter();
  if(HighCounter < (HIGH_NUM - 1))
  {
    HighCounter++;
  }
  High = HighForm[HighCounter];
  dH = High - PLATFORM_HIGH_BASE;
  HighAngle = PlatForm_IK(dH);
  Critical_Exit();
}

void PlatForm_High_Down(void)
{
  Critical_Enter();
  if(HighCounter)
  {
    HighCounter--;
  }
  High = HighForm[HighCounter];
  dH = High - PLATFORM_HIGH_BASE;
  HighAngle = PlatForm_IK(dH);
  Critical_Exit();
}

void PlatForm_High_Add(void)
{
  if(Safe_task_GetAutoTaskFlag())
  {
    Safe_task_ResetAutoTaskFlag();
  }
  Critical_Enter();
  float tmp = High;
  tmp += HIGH_delta * 2;
  High = PLATFORM_CLAMP(PLATFORM_HIGH_MIN,tmp,PLATFORM_HIGH_MAX);
  dH = High - PLATFORM_HIGH_BASE;
  HighAngle = PlatForm_IK(dH);
  Critical_Exit();
}

void PlatForm_High_Sub(void)
{
  if(Safe_task_GetAutoTaskFlag())
  {
    Safe_task_ResetAutoTaskFlag();
  }
  Critical_Enter();
  float tmp = High;
  tmp -= HIGH_delta * 2;
  High = PLATFORM_CLAMP(PLATFORM_HIGH_MIN,tmp,PLATFORM_HIGH_MAX);
  dH = High - PLATFORM_HIGH_BASE;
  HighAngle = PlatForm_IK(dH);
  Critical_Exit();
}

void PlatForm_High_Stop(void)
{
  if(!(Safe_task_GetAutoTaskFlag() || TargetExecute))
  {
    Critical_Enter();
    HighAngle = PV_M.Angle;
    dH = PlatForm_FK(HighAngle);
    High = dH + PLATFORM_HIGH_BASE;
    Critical_Exit();
  }
}

void PlatForm_Conveyor_Forward(void)
{
  SP_c = 10;
}

void PlatForm_Conveyor_Back(void)
{
  SP_c = -10;
}

void PlatForm_Conveyor_Stop(void)
{
  SP_c = 0;
}

void PlatForm_High_Add_delta(void)
{
  Critical_Enter();
  float tmp = High;
  tmp += 0.05;
  High = PLATFORM_CLAMP(PLATFORM_HIGH_MIN,tmp,PLATFORM_HIGH_MAX);
  dH = High - PLATFORM_HIGH_BASE;
  HighAngle = PlatForm_IK(dH);
  TargetExecute = 1;
  Critical_Exit();
}

_Bool PlatForm_High_TargetExecute_SetFlag(void)
{
  TargetExecute = 1;
  return TargetExecute;
}

_Bool PlatForm_High_TargetExecute_ResetFlag(void)
{
  TargetExecute = 0;
  return TargetExecute;
}

_Bool PlatForm_High_TargetExecute_GetFlag(void)
{
  return TargetExecute;
}

_Bool PlatForm_GetHighLicence(void)
{
  Critical_Enter();
  float tmp = (PlatForm_FK(PV_M.Angle) + PLATFORM_HIGH_BASE);
  Critical_Exit();
  return (tmp >= ((PLATFORM_HIGH_MAX + PLATFORM_HIGH_MIN) * 0.5f));
}

void PlatForm_TIM_PeriodCallback(void)
{
  static volatile uint16_t counter = 0;
  static float SP_c_last = 0.0f;
  static float tmpSP_c = 0.0f;
  /* --------------------------------------------- */
  djiMotor_GetPV(M3508_M,&PV_M);
  djiMotor_GetPV(M3508_S,&PV_S);
  djiMotor_GetPV(M2006,&PV_c);
  float tmp = fabsf((PV_M.Angle - PV_S.Angle) / PI);
  /* --------------------------------------------- */
  if(tmp > ERROR_LIM)
  {
  }
  /* --------------------------------------------- */
  else
  {
    if(!counter)
    {
      SP_M[0] = AngleUpdate(PV_M.Angle,HighAngle);
      SP_S[0] = PV_M.Angle;
      tmpSP_c = ((SP_c * Conveyor_Alpha) + ((1.0f - Conveyor_Alpha) * SP_c_last));
      PIDf_Control(SP_M[0],PV_M.Angle,PID_M[0],NULL);
      PIDf_Control(SP_S[0],PV_S.Angle,PID_S[0],NULL);
      PID_GetOutput_f(PID_M[0],&SP_M[1]);
      PID_GetOutput_f(PID_S[0],&SP_S[1]);
      SP_c_last = tmpSP_c;
    }
  /* --------------------------------------------- */
    float tmpSP_M = SP_M[1] * (1.0f - (tmp * INNER_Alpha));
  /* --------------------------------------------- */
    PIDf_Control(tmpSP_M,PV_M.Omega,PID_M[1],&CO_M);
    PIDf_Control(SP_S[1] + tmpSP_M,PV_S.Omega,PID_S[1],&CO_S);
    PIDf_Control(tmpSP_c,PV_c.Omega,PID_c,&CO_c);
  /* --------------------------------------------- */
    Plat_Send_CAN_Cmd(CO_c,CO_M,CO_S,0);
  /* --------------------------------------------- */
    counter++;
    counter %= OUTLOOP_PERIOD;
  }
}

#undef  OUTLOOP_PERIOD
#undef  HIGH_NUM
#undef  HIGH_delta
#undef  Angle_delta
#undef  INNER_Alpha
#undef  ERROR_LIM
#undef  PLATFORM_HIGH_BASE
#undef  PLATFORM_HIGH_MIN
#undef  PLATFORM_HIGH_MAX
#undef  fAlpha
#undef  Conveyor_Alpha
