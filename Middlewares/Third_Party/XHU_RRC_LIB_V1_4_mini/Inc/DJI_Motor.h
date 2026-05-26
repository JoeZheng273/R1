/**
  **********************************************************************************
  * @file    DJI_Motor.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  *
  * @brief   XHU_RRC_LIB 的dji电机部分.
  *
  **********************************************************************************
  * @attention
  * 本人能力有限,请见谅,如有BUG或其他问题,希望你能帮忙改进一下,十分感谢.
  * 欢迎继续扩充.扩充或修改后,欢迎留下名字当作纪念.期待你的加入.
  *
  *
  **********************************************************************************
  * @history
  *  Version        Date                Author              Modification
  *  V1.0.0         2025-12-21          郑琪川              1.0.0  done
  *
  *
  *
  *
  **********************************************************************************
  **/
#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "mod_Motor_Conf.h"
#include "Motor.h"

#if (libUSE_DJIMOTOR)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MOTORTypeDef Motor_Inst_t;
typedef Motor_Inst_t* Motor_t;
typedef void(*djiMotor_Callback)(Motor_t);

/**
  *@brief 枚举类型,电机种类.
  */
typedef enum
{
	GM6020_T = 1,
	M3508_T = 2,
	M2006_T = 3
} MotorType;

/**
  *@brief   在堆区生成电机对象.
  **/
Motor_t New_GM6020_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz);

Motor_t New_M2006_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz);

Motor_t New_M3508_Instance(ESC_ID ID, can_bus can_bus_x, uint16_t samp_freq_hz);

void djiMotor_Set_Flag(Motor_t pMotor, uint16_t FlagBitf);

void djiMotor_ForceChange_Flag(Motor_t pMotor, uint16_t FlagBitf);

void djiMotor_Clear_Flag(Motor_t pMotor, uint16_t FlagBitf);

void djiMotor_Flag_Callback(Motor_t pMotor);

_Bool djiMotor_Set_MotorFlagCallback(uint16_t Flag, djiMotor_Callback Callback);

void djiMotor_Reset_MotorFlagCallback(void);

//CAN 报文处理Callback函数.
void djiMotor_CANFeedbackCallback(ESC_ID ID, can_bus can_bus_x, uint8_t *pData);

void djiMotor_GetPV(Motor_t pMotor_Inst, PV_t *pPV_t);

void djiMotor_Back2Zero(Motor_t pMotor_Inst);

void djiMotor_Set_ecd(Motor_t pMotor_Inst, uint16_t ecd);

void djiMotor_Set_delta_ecd(Motor_t pMotor_Inst, int16_t delta_ecd);

void djiMotor_Set_id(Motor_t pMotor_Inst, ESC_ID id);

void djiMotor_Set_last_ecd(Motor_t pMotor_Inst, uint16_t last_ecd);

void djiMotor_Set_round(Motor_t pMotor_Inst, int round);

void djiMotor_Set_samp_freq(Motor_t pMotor_Inst, uint16_t samp_freq);

void djiMotor_Set_type(Motor_t pMotor_Inst, MotorType type);

void djiMotor_Set_can_bus(Motor_t pMotor_Inst, can_bus CAN_BUS);

void Free_djiMotor_Only(void *p);

/**
  *@brief   释放djiMotor的实例对象.
  *@param   Motor_t  实例对象的指针.
  **/
#define Delete_djiMotor_Instance(ptr)                         \
        do {                                                  \
          extern void Free_djiMotor_Only(void *p);            \
          Free_djiMotor_Only((ptr));                          \
          (ptr) = NULL;                                       \
        } while(0)

        
#ifdef __cplusplus
}
#endif
    
#endif
        
#endif
