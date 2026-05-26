#ifndef __EDGE_COMP_H
#define __EDGE_COMP_H

#include "stdint.h"

#define USE_EC        0

typedef enum
{
  EC_CMD_START  = 0 ,
  EC_CMD_AUTO   = 1 ,
  EC_CMD_REMOTE = 2 ,
} EC_CMD;

void EC_Init(void);

void EC_Control(EC_CMD CMD);

uint8_t EC_GetState(void);

void EC_ChangeState(void);

_Bool EC_GetRxCpltFlag(void);

_Bool EC_SetRxCpltFlag(uint8_t *pRx, uint32_t Size);

/* 会重置Flag,pRx,Size为默认值. 在数据处理完成后自动调用,不需要再调用. */
_Bool EC_ResetRxCpltFlag(void);

void EC_ProcessedData(void);

#endif
