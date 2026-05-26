#include "DJI_Motor.h"
#include "HEAP.h"

#if (libUSE_DJIMOTOR)

struct MOTORTypeDef
{
  uint16_t  _flag;                  /*!<  事件标志位.         */
  uint16_t  _samp_freq;             /*!<  电机数据采样频率.   */
  uint16_t  _last_ecd;              /*!<  上次的编码器反馈值. */
  uint16_t  _ecd;                   /*!<  本次的编码器反馈值. */
  int16_t   _delta_ecd;             /*!<  编码器反馈值增量.   */
  int16_t   _speed_rpm;             /*!<  转子的转速反馈值.   */
  int16_t   _torque_current;        /*!<  转矩电流的反馈值.   */
  int16_t   _temperate;             /*!<  电机温度的反馈值.   */
  int       _round;                 /*!<  转子的转动圈数.     */
  ESC_ID    _id;                    /*!<  电机ID.             */
  can_bus   _can_bus;               /*!<  电机挂载的CAN总线.  */
  MotorType _type;                  /*!<  电机类型.           */
};

static djiMotor_Callback Motor_flag1 = NULL;
static djiMotor_Callback Motor_flag2 = NULL;
static djiMotor_Callback Motor_flag3 = NULL;
static djiMotor_Callback Motor_flag4 = NULL;
static djiMotor_Callback Motor_flag5 = NULL;
static djiMotor_Callback Motor_flag6 = NULL;
static djiMotor_Callback Motor_flag7 = NULL;
static djiMotor_Callback Motor_flag8 = NULL;
static djiMotor_Callback Motor_flag9 = NULL;
static djiMotor_Callback Motor_flag10 = NULL;
static djiMotor_Callback Motor_flag11 = NULL;
static djiMotor_Callback Motor_flag12 = NULL;
static djiMotor_Callback Motor_flag13 = NULL;
static djiMotor_Callback Motor_flag14 = NULL;
static djiMotor_Callback Motor_flag15 = NULL;
static djiMotor_Callback Motor_flag16 = NULL;

static void *Registry[_DJIMOTOR_USE_CAN_BUS_NUMBER_][_DJIMOTOR_MAX_NUMBER_PER_BUS_] = {NULL};

static void* Motor_reg_Add(ESC_ID ID, can_bus can_bus_x)
{
  void *tmp = NULL;
  if(can_bus_x < _DJIMOTOR_USE_CAN_BUS_NUMBER_)
  {
    if(Registry[can_bus_x][ID] == NULL)
    {
      tmp = MALLOC(sizeof(Motor_Inst_t));
      Registry[can_bus_x][ID] = tmp;
    }
    else
    {
      tmp = Registry[can_bus_x][ID];
    }
  }
  return tmp;
}

static void Motor_reg_Delete(ESC_ID ID, can_bus can_bus_x)
{
  if(can_bus_x < _DJIMOTOR_USE_CAN_BUS_NUMBER_)
  {
    Registry[can_bus_x][ID] = NULL;
  }
}

static void Motor_UpdateMeasure(Motor_t pMotor_t, uint8_t *pData)
{
  if((pMotor_t != NULL) && (pData != NULL))
  {
    pMotor_t->_last_ecd = pMotor_t->_ecd;
    pMotor_t->_ecd = (uint16_t)((pData)[0] << 8 | (pData)[1]);
    pMotor_t->_speed_rpm = (uint16_t)((pData)[2] << 8 | (pData)[3]);
    pMotor_t->_torque_current = (uint16_t)((pData)[4] << 8 | (pData)[5]);
    pMotor_t->_temperate = (pData)[6];
    int tDelta = pMotor_t->_ecd - pMotor_t->_last_ecd;
    if(tDelta < -4096)
    {
      pMotor_t->_delta_ecd = tDelta + 8192;
      ++(pMotor_t->_round);
    }
    else if(tDelta > 4096)
    {
      pMotor_t->_delta_ecd = tDelta - 8192;
      --(pMotor_t->_round);
    }
    else
    {
      pMotor_t->_delta_ecd = tDelta;
    }
  }
}

void djiMotor_Set_Flag(Motor_t pMotor, uint16_t FlagBitf)
{
  if(pMotor != NULL)
  {
    pMotor->_flag |= FlagBitf;
  }
}

void djiMotor_ForceChange_Flag(Motor_t pMotor, uint16_t FlagBitf)
{
  if(pMotor != NULL)
  {
    pMotor->_flag = FlagBitf;
  }
}

void djiMotor_Clear_Flag(Motor_t pMotor, uint16_t FlagBitf)
{
  if(pMotor != NULL)
  {
    pMotor->_flag &= (~FlagBitf);
  }
}

void djiMotor_Flag_Callback(Motor_t pMotor)
{
  if(pMotor != NULL)
  {
    if(pMotor->_flag)
    {
      for(int i=0;i<16;i++)
      {
        if(pMotor->_flag)
        {
          switch((pMotor->_flag) & (1<<i))
          {
            case 0x0001 : if(Motor_flag1 != NULL){Motor_flag1(pMotor);} break;
            case 0x0002 : if(Motor_flag2 != NULL){Motor_flag2(pMotor);} break;
            case 0x0004 : if(Motor_flag3 != NULL){Motor_flag3(pMotor);} break;
            case 0x0008 : if(Motor_flag4 != NULL){Motor_flag4(pMotor);} break;
            case 0x0010 : if(Motor_flag5 != NULL){Motor_flag5(pMotor);} break;
            case 0x0020 : if(Motor_flag6 != NULL){Motor_flag6(pMotor);} break;
            case 0x0040 : if(Motor_flag7 != NULL){Motor_flag7(pMotor);} break;
            case 0x0080 : if(Motor_flag8 != NULL){Motor_flag8(pMotor);} break;
            case 0x0100 : if(Motor_flag9 != NULL){Motor_flag9(pMotor);} break;
            case 0x0200 : if(Motor_flag10 != NULL){Motor_flag10(pMotor);} break;
            case 0x0400 : if(Motor_flag11 != NULL){Motor_flag11(pMotor);} break;
            case 0x0800 : if(Motor_flag12 != NULL){Motor_flag12(pMotor);} break;
            case 0x1000 : if(Motor_flag13 != NULL){Motor_flag13(pMotor);} break;
            case 0x2000 : if(Motor_flag14 != NULL){Motor_flag14(pMotor);} break;
            case 0x4000 : if(Motor_flag15 != NULL){Motor_flag15(pMotor);} break;
            case 0x8000 : if(Motor_flag16 != NULL){Motor_flag16(pMotor);} break;
            default : break;
          }
          pMotor->_flag &= (~(1<<i));
        }
        else
        {
          return;
        }
      }
    }
  }
}

void Free_djiMotor_Only(void *p)
{
  if(p != NULL)
  {
    Motor_t ptr = (Motor_t)p;
    Motor_reg_Delete(ptr->_id,ptr->_can_bus);
    Free_Only(ptr);
  }
}

_Bool djiMotor_Set_MotorFlagCallback(uint16_t Flag, djiMotor_Callback Callback)
{
  _Bool Ret = 0;
  switch(Flag)
  {
    case 0x0001 : Motor_flag1 = Callback; Ret = 1; break;
    case 0x0002 : Motor_flag2 = Callback; Ret = 1; break;
    case 0x0004 : Motor_flag3 = Callback; Ret = 1; break;
    case 0x0008 : Motor_flag4 = Callback; Ret = 1; break;
    case 0x0010 : Motor_flag5 = Callback; Ret = 1; break;
    case 0x0020 : Motor_flag6 = Callback; Ret = 1; break;
    case 0x0040 : Motor_flag7 = Callback; Ret = 1; break;
    case 0x0080 : Motor_flag8 = Callback; Ret = 1; break;
    case 0x0100 : Motor_flag9 = Callback; Ret = 1; break;
    case 0x0200 : Motor_flag10 = Callback; Ret = 1; break;
    case 0x0400 : Motor_flag11 = Callback; Ret = 1; break;
    case 0x0800 : Motor_flag12 = Callback; Ret = 1; break;
    case 0x1000 : Motor_flag13 = Callback; Ret = 1; break;
    case 0x2000 : Motor_flag14 = Callback; Ret = 1; break;
    case 0x4000 : Motor_flag15 = Callback; Ret = 1; break;
    case 0x8000 : Motor_flag16 = Callback; Ret = 1; break;
    default : break;
  }
  return Ret;
}

void djiMotor_Reset_MotorFlagCallback(void)
{
  Motor_flag1 = NULL;
  Motor_flag2 = NULL;
  Motor_flag3 = NULL;
  Motor_flag4 = NULL;
  Motor_flag5 = NULL;
  Motor_flag6 = NULL;
  Motor_flag7 = NULL;
  Motor_flag8 = NULL;
  Motor_flag9 = NULL;
  Motor_flag10 = NULL;
  Motor_flag11 = NULL;
  Motor_flag12 = NULL;
  Motor_flag13 = NULL;
  Motor_flag14 = NULL;
  Motor_flag15 = NULL;
  Motor_flag16 = NULL;
}

Motor_t New_GM6020_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz)
{
  Motor_t tmp = (Motor_t)Motor_reg_Add(ID,can_bus_x);
  if(tmp != NULL)
  {
    tmp->_delta_ecd = 0;
    tmp->_ecd = 0;
    tmp->_id = ID;
    tmp->_last_ecd = 0;
    tmp->_round = 0;
    tmp->_samp_freq = samp_freq_hz;
    tmp->_speed_rpm = 0;
    tmp->_temperate = 0;
    tmp->_torque_current = 0;
    tmp->_type = GM6020_T;
    tmp->_can_bus = can_bus_x;
  }
  return tmp;
}

Motor_t New_M2006_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz)
{
	Motor_t tmp = (Motor_t)Motor_reg_Add(ID,can_bus_x);
  if(tmp != NULL)
  {
    tmp->_delta_ecd = 0;
    tmp->_ecd = 0;
    tmp->_id = ID;
    tmp->_last_ecd = 0;
    tmp->_round = 0;
    tmp->_samp_freq = samp_freq_hz;
    tmp->_speed_rpm = 0;
    tmp->_temperate = 0;
    tmp->_torque_current = 0;
    tmp->_type = M2006_T;
    tmp->_can_bus = can_bus_x;
  }
  return tmp;
}

Motor_t New_M3508_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz)
{
	Motor_t tmp = (Motor_t)Motor_reg_Add(ID,can_bus_x);
  if(tmp != NULL)
  {
    tmp->_delta_ecd = 0;
    tmp->_ecd = 0;
    tmp->_id = ID;
    tmp->_last_ecd = 0;
    tmp->_round = 0;
    tmp->_samp_freq = samp_freq_hz;
    tmp->_speed_rpm = 0;
    tmp->_temperate = 0;
    tmp->_torque_current = 0;
    tmp->_type = M3508_T;
    tmp->_can_bus = can_bus_x;
  }
  return tmp;
}

void djiMotor_Back2Zero(Motor_t pMotor_Inst)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_delta_ecd = 0;
    pMotor_Inst->_ecd = 0;
    pMotor_Inst->_last_ecd = 0;
    pMotor_Inst->_round = 0;
  }
}

void djiMotor_Set_ecd(Motor_t pMotor_Inst, uint16_t ecd)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_ecd = (ecd > 8191 ? 8191 : ecd);
  }
}

void djiMotor_Set_delta_ecd(Motor_t pMotor_Inst, int16_t delta_ecd)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_delta_ecd = (delta_ecd > 8191 ? 8191 : delta_ecd);
  }
}

void djiMotor_Set_id(Motor_t pMotor_Inst, ESC_ID id)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_id = id;
  }
}

void djiMotor_Set_last_ecd(Motor_t pMotor_Inst, uint16_t last_ecd)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_last_ecd = (last_ecd > 8191 ? 8191 : last_ecd);
  }
}

void djiMotor_Set_round(Motor_t pMotor_Inst, int round)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_round = round;
  }
}

void djiMotor_Set_samp_freq(Motor_t pMotor_Inst, uint16_t samp_freq)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_samp_freq = samp_freq;
  }
}

void djiMotor_Set_type(Motor_t pMotor_Inst, MotorType type)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_type = type;
  }
}

void djiMotor_Set_can_bus(Motor_t pMotor_Inst, can_bus CAN_BUS)
{
  if(pMotor_Inst != NULL)
  {
    pMotor_Inst->_can_bus = CAN_BUS;
  }
}

void djiMotor_GetPV(Motor_t pMotor_Inst, PV_t *pPV_t)
{
  if((pMotor_Inst != NULL) && (pPV_t != NULL))
  {
    switch(pMotor_Inst->_type)
    {
      case GM6020_T :
        // ((float)ecd/8192)*2*pi = (float)ecd * 0.0007669903939f;
        //  Round * (2*pi)
        pPV_t->Angle = ((float)pMotor_Inst->_round * 6.283185307f) +
          ((float)pMotor_Inst->_ecd * 0.0007669903939f);
        // rad/s = ((float)RPM/60)*2*pi , rad/s(输出) = (rad/s(转子));
        pPV_t->Omega = (float)pMotor_Inst->_speed_rpm * 0.1047197551f;
        // (0,3)(A) 对应 (0,16384) , 3/16384 = 0.000183105f;
        pPV_t->Current = (float)pMotor_Inst->_torque_current * 0.000183105f;
        // T = Kt * I , Kt = 0.741 , (3 * 0.741)/16384 = 0.000135681f;
        pPV_t->Torque = (float)pMotor_Inst->_torque_current * 0.000135681f;
        pPV_t->Temperate = (float)pMotor_Inst->_temperate;
        pPV_t->Omega_ecd = ((float)pMotor_Inst->_delta_ecd *
          (float)pMotor_Inst->_samp_freq) * 0.0007669903939f;
        break;
      case M3508_T  :
        // ((float)ecd/8192)*2*pi/(3591/187) = (float)ecd * 0.0000399407f;
        //  Round * (2*pi)/(3591/187)
        pPV_t->Angle = ((float)pMotor_Inst->_round * 0.3271945565f) +
          ((float)pMotor_Inst->_ecd * 0.0000399407f);
        // rad/s = ((float)RPM/60)*2*pi , rad/s(输出) = (rad/s(转子))/(3591/187);
        pPV_t->Omega = (float)pMotor_Inst->_speed_rpm * 0.00545324f;
        // (0,20)(A) 对应 (0,16384) , 20/16384 = 0.00122070f;
        pPV_t->Current = (float)pMotor_Inst->_torque_current * 0.00122070f;
        // T = Kt * I , Kt = 0.3 , (20 * 0.3)/16384 = 0.000366211f;
        pPV_t->Torque = (float)pMotor_Inst->_torque_current * 0.000366211f;
        pPV_t->Temperate = (float)pMotor_Inst->_temperate;
        pPV_t->Omega_ecd = ((float)pMotor_Inst->_delta_ecd *
          (float)pMotor_Inst->_samp_freq) * 0.0000399407f;
        break;
      case M2006_T  :
        // ((float)ecd/8192)*2*pi/36 = (float)ecd * 0.0000213053f;
        //  Round * (2*pi)/36
        pPV_t->Angle = ((float)pMotor_Inst->_round * 0.1745329252f) +
          ((float)pMotor_Inst->_ecd * 0.0000213053f);
        // rad/s = ((float)RPM/60)*2*pi , rad/s(输出) = (rad/s(转子))/36;
        pPV_t->Omega = (float)pMotor_Inst->_speed_rpm * 0.00290888f;
        // (0,10)(A) 对应 (0,10000) , 10/10000 = 0.001f;
        pPV_t->Current = (float)pMotor_Inst->_torque_current * 0.001f;
        // T = Kt * I , Kt = 0.18 , (10 * 0.18)/10000 = 0.00018f;
        pPV_t->Torque = (float)pMotor_Inst->_torque_current * 0.00018f;
        pPV_t->Temperate = (float)pMotor_Inst->_temperate;
        pPV_t->Omega_ecd = ((float)pMotor_Inst->_delta_ecd *
          (float)pMotor_Inst->_samp_freq) * 0.0000213053f;
        break;
      default : break;
    }
  }
}

void djiMotor_CANFeedbackCallback(ESC_ID ID, can_bus can_bus_x, uint8_t *pData)
{
  if((can_bus_x < _DJIMOTOR_USE_CAN_BUS_NUMBER_)&&(pData != NULL))
  {
    void *tmp = Registry[can_bus_x][ID];
    if(tmp != NULL)
    {
      Motor_UpdateMeasure(tmp,pData);
    }
  }
}

#endif
