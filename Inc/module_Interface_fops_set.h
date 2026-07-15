/**
  ******************************************************************************
  * @file           : module_Interface_fops_set.h
  * @brief          : 一些模块接口的虚函数表
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __MODULE_INTERFACE_FOPS_SET_H
#define __MODULE_INTERFACE_FOPS_SET_H

/* ----------------------------------------------------- */
#include "PROJECT_MACRO.h"

/* ----------------------------------------------------- */
#undef  USER
#define USER            USER_alpha

/* ----------------------------------------------------- */
typedef struct
{
  void(*Set_SP)(float,float,float);
  void(*Updata_PV)(float*,unsigned char);
} Chassis_ItfTypeDef;

typedef struct
{
  void(*KeyOP_Super)(void);
  void(*KeyOP_DUp_T)(void);
  void(*KeyOP_DDn_T)(void);
  void(*KeyOP_DLf_T)(void);
  void(*KeyOP_DRt_T)(void);
  void(*KeyOP_AUp_T)(void);
  void(*KeyOP_ADn_T)(void);
  void(*KeyOP_ALf_T)(void);
  void(*KeyOP_ARt_T)(void);
  void(*KeyOP_BUp_T)(void);
  void(*KeyOP_BDn_T)(void);
  void(*KeyOP_BLf_T)(void);
  void(*KeyOP_BRt_T)(void);
  void(*KeyOP_CUp_T)(void);
  void(*KeyOP_CDn_T)(void);
  void(*KeyOP_CLf_T)(void);
  void(*KeyOP_CRt_T)(void);
  void(*KeyOP_Up_N)(void);
  void(*KeyOP_Dn_N)(void);
  void(*KeyOP_Lf_N)(void);
  void(*KeyOP_Rt_N)(void);
} KeyMapping_ItfTypeDef;

/* ----------------------------------------------------- */
extern const Chassis_ItfTypeDef Chassis_Interface_fops;
extern const KeyMapping_ItfTypeDef KeyMap_Interface_fops;

#endif
