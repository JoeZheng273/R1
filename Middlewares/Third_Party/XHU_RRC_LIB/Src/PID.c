#include "PID.h"
#include "string.h"
#include "limits.h"

#undef    __PID_EN_Bitf
#undef    __PID_MUTEX_LOCK
#undef    __PID_P_Enable
#undef    __PID_I_Enable
#undef    __PID_D_Enable
#undef    __PID_F_Enable
#undef    __PID_Float_Epsilon
#undef    __PID_recp_Float_Epsilon

#define   __PID_EN_Bitf                     0x0F000000U
#define   __PID_MUTEX_LOCK                  0x00100000U
#define   __PID_P_Enable                    0x01000000U
#define   __PID_I_Enable                    0x02000000U
#define   __PID_D_Enable                    0x04000000U
#define   __PID_F_Enable                    0x08000000U
#define   __PID_Float_Epsilon               1.0E-6f
#define   __PID_recp_Float_Epsilon          1.0E6f

/*---------------------------------------------------*/
struct PID_InstanceTypeDef
{
  unsigned int _Flag;                      /*!<  各项的使能标志位          */
	float _Kf;                               /*!<  前馈项系数                */
	float _Kp;                               /*!<  比例项系数                */
	float _Ki;                               /*!<  积分项系数                */
	float _Kd;                               /*!<  微分项系数                */
  float _Alpha;                            /*!<  一阶滤波的系数            */
  float _Epsilon;                          /*!<  误差死区                  */
	float _Summax;                           /*!<  积分限幅                  */
	float _Outputmax;                        /*!<  输出限幅                  */
	float _SumOp_region;                     /*!<  积分有效区间              */
  float _dt;                               /*!<  采样时间,unit: s          */
  float _recp_dt;                          /*!<  采样时间的倒数,unit: 1/s  */
  Forward_func_t  _Forward;                /*!<  前馈回调函数              */
  SumDecay_func_t _SumDecay;               /*!<  积分衰减回调函数          */
  float _Sum;                              /*!<  积分求和                  */
	float _last_Err;                         /*!<  上次误差                  */
	float _Err;                              /*!<  本次误差                  */
	float _Change;                           /*!<  两次误差之间的改变量      */
  float _Change_filt;                      /*!<  一阶滤波后的Change        */
	float _Output_f;                         /*!<  原始计算得到的输出值      */
	float _F_term;                           /*!<  前馈项                    */
	float _P_term;                           /*!<  比例项                    */
	float _I_term;                           /*!<  积分项                    */
	float _D_term;                           /*!<  微分项                    */
};

/*-------------------------------------------------------*/
static float DefaultForwardInit(const float Target, const PID_Inst_t* const pInst)
{ return 0.0f;  }

static float DefaultSumDecayInit(const PID_Inst_t* const pInst)
{ return 0.0f;  }

static inline float PID_CLAMP(float value, float max_abs)
{
  if((!isfinite(value))||(!isfinite(max_abs))||(max_abs < 0.0f))
  { return 0.0f; }
  else
  {
    if(value >= max_abs)
    { return max_abs; }
    else if(value <= -max_abs)
    { return -max_abs; }
    else
    { return value; }
  }
}

static inline int PID_INT_CLAMP(float value)
{
  if(isfinite(value))
  {
    float Value = roundf(value);
    if(Value >= (float)INT_MAX)
    { return INT_MAX; }
    else if(Value <= (float)INT_MIN)
    { return INT_MIN; }
    else
    { return (int)Value;  }
  }
  else
  { return 0; }
}

/*---------------------------------------------------------*/
PID_Status PIDf_Control(float SP, float PV, PID_t pPID_Instance, int *pCO)
{
  /* 定义变量并初始化. */
  PID_Status tReturn = PID_OK;  // 状态变量.
  float Tmp = 0.0f;
  /* 检查参数,如果指针为NULL,或float型参数有问题,返回错误码. */
  if((pPID_Instance == NULL)||(!isfinite(SP))||(!isfinite(PV)))
  {
    if(pCO != NULL)
    { *pCO = 0; } // 如果pCO不为空,则清空输出.
    return PID_ERR_0;
  }
  /* 参数检验无误. */
  else
  {
    /* 检查互斥锁 */
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_WARN_3;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      /* 检查使能,如果使能位全为空,清空输出,并返回错误码. */
      if(!(pPID_Instance->_Flag & __PID_EN_Bitf))
      {
        if(pCO != NULL)
        { *pCO = 0; }
        pPID_Instance->_Output_f = 0.0f;
        (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
        return PID_ERR_1;
      }
      else
      {
        /* 以下开始为PID控制的核心部分. */
        pPID_Instance->_Err = SP - PV;  // 计算误差.
        /* 误差死区. */
        if((pPID_Instance->_Err <= pPID_Instance->_Epsilon)
            &&(pPID_Instance->_Err >= -pPID_Instance->_Epsilon))
        { pPID_Instance->_Err = 0.0f; }
        /* 检验P项参数有效与使能位. */
        if((pPID_Instance->_Kp >= __PID_Float_Epsilon)
            &&(pPID_Instance->_Flag & __PID_P_Enable))
        {
          pPID_Instance->_P_term = pPID_Instance->_Kp * pPID_Instance->_Err;
          Tmp += pPID_Instance->_P_term;
        }
        /* 检验F项参数有效与使能位. */
        if((pPID_Instance->_Kf >= __PID_Float_Epsilon)
            &&(pPID_Instance->_Flag & __PID_F_Enable))
        {
          /* 检验自定义前馈回调函数指针 */
          if(pPID_Instance->_Forward != NULL)
          {
            pPID_Instance->_F_term = (pPID_Instance->_Forward(SP,pPID_Instance))
             * pPID_Instance->_Kf;
            Tmp += pPID_Instance->_F_term;
          }
          else
          {
            /* 清空前馈项. */
            pPID_Instance->_F_term = 0.0f;
            /* 前馈项失能. */
            (pPID_Instance->_Flag) &= (~__PID_F_Enable);
            tReturn |= PID_WARN_1;
          }
        }
        /* 检验I项参数有效与使能位. */
        if((pPID_Instance->_Ki >= __PID_Float_Epsilon)
            &&(pPID_Instance->_Flag & __PID_I_Enable))
        {
          /* 积分有效区间. */
          if((pPID_Instance->_Err <= pPID_Instance->_SumOp_region)
              &&(pPID_Instance->_Err >= -pPID_Instance->_SumOp_region))
          { pPID_Instance->_Sum += pPID_Instance->_Err;  }
          else
          {
            /* 检验自定义积分衰减回调函数指针. */
            if(pPID_Instance->_SumDecay != NULL)
            { pPID_Instance->_Sum = pPID_Instance->_SumDecay(pPID_Instance); }
            else
            {
              if((pPID_Instance->_Sum < 1.0f)
                  &&(pPID_Instance->_Sum > -1.0f))
              { pPID_Instance->_Sum = 0.0f; }
              else
              { pPID_Instance->_Sum *= 0.5f; }
              tReturn |= PID_WARN_2;
            }
          }
          /* 积分限幅. */
          pPID_Instance->_Sum =
          PID_CLAMP(pPID_Instance->_Sum , pPID_Instance->_Summax);
          pPID_Instance->_I_term = pPID_Instance->_Ki * 
          pPID_Instance->_Sum * pPID_Instance->_dt;
          Tmp += pPID_Instance->_I_term;
        }
        /* 检验D项参数有效与使能位. */
        if((pPID_Instance->_Kd >= __PID_Float_Epsilon)
            && (pPID_Instance->_Flag & __PID_D_Enable))
        {
          pPID_Instance->_Change = pPID_Instance->_Err - pPID_Instance->_last_Err;
          /* 检验滤波系数有效性. */
          if((pPID_Instance->_Alpha > __PID_Float_Epsilon)
              &&(pPID_Instance->_Alpha < 1.0f))
          {
            pPID_Instance->_Change_filt =
            ((1.0f - pPID_Instance->_Alpha) * pPID_Instance->_Change_filt) +
            (pPID_Instance->_Alpha * pPID_Instance->_Change);
            pPID_Instance->_D_term = pPID_Instance->_Kd *
            pPID_Instance->_Change_filt * pPID_Instance->_recp_dt;
          }
          else
          {
            pPID_Instance->_D_term = pPID_Instance->_Kd * 
            pPID_Instance->_Change * pPID_Instance->_recp_dt;
            tReturn |= PID_WARN_0;
          }
          Tmp += pPID_Instance->_D_term;
          pPID_Instance->_last_Err = pPID_Instance->_Err;
        }
        /* 输出限幅. */
        pPID_Instance->_Output_f = PID_CLAMP(Tmp , pPID_Instance->_Outputmax);
        /* 转化为整型,安全输出 */
        if(pCO != NULL)
        { *pCO = PID_INT_CLAMP(pPID_Instance->_Output_f); }
        (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
        return tReturn;
      }
    }
  }
}

/*---------------------------------------------------------*/
size_t PID_Sizeof_Instance(void)
{
  return (sizeof(struct PID_InstanceTypeDef));
}

void PID_InstanceReset(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    pPID_Instance->_Flag = 0U;
    pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
    pPID_Instance->_Outputmax = 0.0f;
    pPID_Instance->_Output_f = 0.0f;
    pPID_Instance->_Alpha = 0.1f;
    pPID_Instance->_Change = 0.0f;
    pPID_Instance->_Change_filt = 0.0f;
    pPID_Instance->_dt = 1.0f;
    pPID_Instance->_D_term = 0.0f;
    pPID_Instance->_Epsilon = 0.0f;
    pPID_Instance->_Err = 0.0f;
    pPID_Instance->_Forward = DefaultForwardInit;
    pPID_Instance->_F_term = 0.0f;
    pPID_Instance->_I_term = 0.0f;
    pPID_Instance->_Kd = 0.0f;
    pPID_Instance->_Kf = 0.0f;
    pPID_Instance->_Ki = 0.0f;
    pPID_Instance->_Kp = 0.0f;
    pPID_Instance->_last_Err = 0.0f;
    pPID_Instance->_P_term = 0.0f;
    pPID_Instance->_recp_dt = 1.0f;
    pPID_Instance->_SumDecay = DefaultSumDecayInit;
    pPID_Instance->_Summax = 0.0f;
    pPID_Instance->_SumOp_region = 0.0f;
    pPID_Instance->_Sum = 0.0f;
    (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
  }
}

void PID_MutexForceUnLock(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
  }
}

PID_Status PID_InstanceSafeReset(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Alpha = 0.1f;
      pPID_Instance->_Change = 0.0f;
      pPID_Instance->_Change_filt = 0.0f;
      pPID_Instance->_dt = 1.0f;
      pPID_Instance->_D_term = 0.0f;
      pPID_Instance->_Epsilon = 0.0f;
      pPID_Instance->_Err = 0.0f;
      pPID_Instance->_Flag = 0U;
      pPID_Instance->_Forward = DefaultForwardInit;
      pPID_Instance->_F_term = 0.0f;
      pPID_Instance->_I_term = 0.0f;
      pPID_Instance->_Kd = 0.0f;
      pPID_Instance->_Kf = 0.0f;
      pPID_Instance->_Ki = 0.0f;
      pPID_Instance->_Kp = 0.0f;
      pPID_Instance->_last_Err = 0.0f;
      pPID_Instance->_Outputmax = 0.0f;
      pPID_Instance->_Output_f = 0.0f;
      pPID_Instance->_P_term = 0.0f;
      pPID_Instance->_recp_dt = 1.0f;
      pPID_Instance->_SumDecay = DefaultSumDecayInit;
      pPID_Instance->_Summax = 0.0f;
      pPID_Instance->_SumOp_region = 0.0f;
      pPID_Instance->_Sum = 0.0f;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

void PID_Instance_Init(void *p)
{
  PID_Inst_t *ptr = (PID_Inst_t*)p;
  PID_InstanceReset(ptr);
}

/*----------------  Instance   -----------------------*/
PID_t New_PID_Instance(void)
{
  PID_t tmp = (PID_t)MALLOC(sizeof(PID_Inst_t));
  if(tmp == NULL)
  {
    return NULL;
  }
  else
  {
    PID_InstanceReset(tmp);
    return tmp;
  }
}

/*----------------------  SetParam   ---------------------*/
PID_Status PID_SetBasicParam(PID_Inst_t *pPID_Instance, const PID_Basic_Param_t *pParam_t)
{
  if((pPID_Instance != NULL) && (pParam_t != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Alpha = pParam_t->Alpha;
      pPID_Instance->_Epsilon = pParam_t->Epsilon;
      pPID_Instance->_Kd = pParam_t->Kd;
      pPID_Instance->_Kf = pParam_t->Kf;
      pPID_Instance->_Ki = pParam_t->Ki;
      pPID_Instance->_Kp = pParam_t->Kp;
      pPID_Instance->_Outputmax = pParam_t->Outputmax;
      pPID_Instance->_Summax = pParam_t->Summax;
      pPID_Instance->_SumOp_region = pParam_t->SumOp_region;
      pPID_Instance->_Flag |= __PID_EN_Bitf;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Alpha(PID_Inst_t *pPID_Instance, const float Alpha)
{
  if((pPID_Instance != NULL)
      && (isfinite(Alpha)) && (Alpha >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Alpha = Alpha;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Epsilon(PID_Inst_t *pPID_Instance, const float Epsilon)
{
  if((pPID_Instance != NULL)
      && (isfinite(Epsilon)) && (Epsilon >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Epsilon = Epsilon;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Kd(PID_Inst_t *pPID_Instance, const float Kd)
{
  if((pPID_Instance != NULL)
      && (isfinite(Kd)) && (Kd >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Kd = Kd; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Kf(PID_Inst_t *pPID_Instance, const float Kf)
{
  if((pPID_Instance != NULL)
      && (isfinite(Kf)) && (Kf >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Kf = Kf; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Ki(PID_Inst_t *pPID_Instance, const float Ki)
{
  if((pPID_Instance != NULL)
      && (isfinite(Ki)) && (Ki >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Ki = Ki; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Kp(PID_Inst_t *pPID_Instance, const float Kp)
{
  if((pPID_Instance != NULL)
      && (isfinite(Kp)) && (Kp >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Kp = Kp; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Outputmax(PID_Inst_t *pPID_Instance, const float Outputmax)
{
  if((pPID_Instance != NULL)
      && (isfinite(Outputmax)) && (Outputmax >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Outputmax = Outputmax; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Summax(PID_Inst_t *pPID_Instance, const float Summax)
{
  if((pPID_Instance != NULL)
      && (isfinite(Summax)) && (Summax >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Summax = Summax; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_SumOp_region(PID_Inst_t *pPID_Instance, const float SumOp_region)
{
  if((pPID_Instance != NULL)
      && (isfinite(SumOp_region)) && (SumOp_region >= 0))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_SumOp_region = SumOp_region; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

/* ----------------------------------------------------- */
PID_Status PID_SetParam_dt(PID_Inst_t *pPID_Instance, const float dt)
{
  if((pPID_Instance != NULL) && (dt < __PID_recp_Float_Epsilon)
      && (isfinite(dt)) && (dt > __PID_Float_Epsilon))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_dt = dt;
      pPID_Instance->_recp_dt = 1.0f/dt;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_ResetParam_dt(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_dt = 1.0f;
      pPID_Instance->_recp_dt = 1.0f;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_SumDecay(PID_Inst_t *pPID_Instance, SumDecay_func_t SumDecay)
{
  if((pPID_Instance != NULL) && (SumDecay != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_SumDecay = SumDecay; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_ResetParam_SumDecay(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_SumDecay = DefaultSumDecayInit; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_SetParam_Forward(PID_Inst_t *pPID_Instance, Forward_func_t Forward)
{
  if((pPID_Instance != NULL) && (Forward != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Forward = Forward; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_ResetParam_Forward(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Forward = DefaultForwardInit; 
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}
/* ---------------------------------------------------- */

PID_Status PID_ResetBasicParam(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Alpha = 0.1f;
      pPID_Instance->_Epsilon = 0.0f;
      pPID_Instance->_Kd = 0.0f;
      pPID_Instance->_Kf = 0.0f;
      pPID_Instance->_Ki = 0.0f;
      pPID_Instance->_Kp = 0.0f;
      pPID_Instance->_Outputmax = 0.0f;
      pPID_Instance->_Summax = 0.0f;
      pPID_Instance->_SumOp_region = 0.0f;
      pPID_Instance->_Flag &= (~__PID_EN_Bitf);
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

/*--------------------  GetParam   -------------------------*/
PID_Status PID_GetBasicParam(const PID_Inst_t *pPID_Instance, PID_Basic_Param_t *pParam_t)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pParam_t != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    pParam_t->Alpha = pPID_Instance->_Alpha;
    pParam_t->Epsilon = pPID_Instance->_Epsilon;
    pParam_t->Kd = pPID_Instance->_Kd;
    pParam_t->Kf = pPID_Instance->_Kf;
    pParam_t->Ki = pPID_Instance->_Ki;
    pParam_t->Kp = pPID_Instance->_Kp;
    pParam_t->Outputmax = pPID_Instance->_Outputmax;
    pParam_t->Summax = pPID_Instance->_Summax;
    pParam_t->SumOp_region = pPID_Instance->_SumOp_region;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Alpha(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Alpha;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Epsilon(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Epsilon;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Kd(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Kd;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Kf(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Kf;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Ki(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Ki;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Kp(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Kp;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Outputmax(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Outputmax;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_Summax(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Summax;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_SumOp_region(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_SumOp_region;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetParam_dt(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_dt;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

_Bool PID_CmpParam_SumDecay(const PID_Inst_t *pPID_Instance, SumDecay_func_t SumDecay)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_SumDecay == SumDecay)
    { return true; }
    else
    { return false; }
  }
  else
  { return false; }
}

_Bool PID_CmpParam_Forward(const PID_Inst_t *pPID_Instance, Forward_func_t Forward)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Forward == Forward)
    { return true; }
    else
    { return false; }
  }
  else
  { return false; }
}

/*----------  ENABLE  -----------------------------*/
void PID_ENABLE_ALL(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag |= __PID_EN_Bitf; }
}

void PID_ENABLE_P(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag |= __PID_P_Enable; }
}

void PID_ENABLE_I(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag |= __PID_I_Enable; }
}

void PID_ENABLE_D(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag |= __PID_D_Enable; }
}

void PID_ENABLE_F(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag |= __PID_F_Enable; }
}

void PID_DISABLE_ALL(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag &= (~__PID_EN_Bitf); }
}

void PID_DISABLE_P(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag &= (~__PID_P_Enable); }
}

void PID_DISABLE_I(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag &= (~__PID_I_Enable); }
}

void PID_DISABLE_D(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag &= (~__PID_D_Enable); }
}

void PID_DISABLE_F(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {pPID_Instance->_Flag &= (~__PID_F_Enable); }
}

/*--------------------  GetContainer ------------------------*/
PID_Status PID_GetContainer(const PID_Inst_t *pPID_Instance, PID_Container_t *pContainer_t)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pContainer_t != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    pContainer_t->Change = pPID_Instance->_Change;
    pContainer_t->Change_filt = pPID_Instance->_Change_filt;
    pContainer_t->D_term = pPID_Instance->_D_term;
    pContainer_t->Err = pPID_Instance->_Err;
    pContainer_t->F_term = pPID_Instance->_F_term;
    pContainer_t->I_term = pPID_Instance->_I_term;
    pContainer_t->last_Err = pPID_Instance->_last_Err;
    pContainer_t->Output_f = pPID_Instance->_Output_f;
    pContainer_t->P_term = pPID_Instance->_P_term;
    pContainer_t->Sum = pPID_Instance->_Sum;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_Change(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Change;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_Change_filt(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Change_filt;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_D_term(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_D_term;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_Err(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Err;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_F_term(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_F_term;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_I_term(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_I_term;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_last_Err(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_last_Err;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetOutput_f(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Output_f;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_P_term(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_P_term;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_GetContainer_Sum(const PID_Inst_t *pPID_Instance, float *pResult)
{
  PID_Status tmp = PID_OK;
  if((pPID_Instance != NULL) && (pResult != NULL))
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    *pResult = pPID_Instance->_Sum;
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    { tmp = PID_WARN; }
    return tmp;
  }
  else
  {
    if(pResult != NULL)
    { *pResult = 0.0f;  }
    return PID_ERR_0;
  }
}

PID_Status PID_ResetContainer(PID_Inst_t *pPID_Instance)
{
  if(pPID_Instance != NULL)
  {
    if(pPID_Instance->_Flag & __PID_MUTEX_LOCK)
    {
      return PID_BUSY;  //  互斥锁已锁,什么也不做,直接返回.
    }
    else
    {
      pPID_Instance->_Flag |= __PID_MUTEX_LOCK; //  互斥锁上锁.
      pPID_Instance->_Change = 0.0f;
      pPID_Instance->_Change_filt = 0.0f;
      pPID_Instance->_D_term = 0.0f;
      pPID_Instance->_Err = 0.0f;
      pPID_Instance->_F_term = 0.0f;
      pPID_Instance->_I_term = 0.0f;
      pPID_Instance->_last_Err = 0.0f;
      pPID_Instance->_Output_f = 0.0f;
      pPID_Instance->_P_term = 0.0f;
      pPID_Instance->_Sum = 0.0f;
      (pPID_Instance->_Flag) &= (~__PID_MUTEX_LOCK);  //  互斥锁解锁.
      return PID_OK;
    }
  }
  else
  {
    return PID_ERR_0;
  }
}

#undef    __PID_EN_Bitf
#undef    __PID_MUTEX_LOCK
#undef    __PID_P_Enable
#undef    __PID_I_Enable
#undef    __PID_D_Enable
#undef    __PID_F_Enable
#undef    __PID_Float_Epsilon
#undef    __PID_recp_Float_Epsilon
