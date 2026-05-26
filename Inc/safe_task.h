#ifndef __SAFE_TASK_H
#define __SAFE_TASK_H

#include "stdint.h"

void Safe_task_Update_LimFlag(void);

_Bool Safe_task_Get_LimFlag(uint16_t Key_ID);

void Safe_task_TIM_PeriodCallback(void);

void Safe_task_Back_to_zero(void);

_Bool Saft_task_GetAutoTaskFlag(void);

_Bool Saft_task_ResetAutoTaskFlag(void);

#endif
