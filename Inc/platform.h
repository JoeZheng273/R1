#ifndef __PLATFORM_H
#define __PLATFORM_H

//extern volatile _Bool WorkingFlag;

void PlatForm_Init(void);

void PlatForm_BackToZero_Start(void);

void PlatForm_BackToZero_End(void);

void PlatForm_Conveyor_Forward(void);

void PlatForm_Conveyor_Back(void);

void PlatForm_Conveyor_Stop(void);

void PlatForm_High_Add(void);

void PlatForm_High_Sub(void);

void PlatForm_High_Stop(void);

void PlatForm_High_Up(void);

void PlatForm_High_Down(void);

void PlatForm_TIM_PeriodCallback(void);

void PlatForm_High_Add_delta(void);

_Bool PlatForm_High_TargetExecute_SetFlag(void);

_Bool PlatForm_High_TargetExecute_ResetFlag(void);

_Bool PlatForm_High_TargetExecute_GetFlag(void);

#endif

