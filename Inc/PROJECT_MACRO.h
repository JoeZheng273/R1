/**
  ******************************************************************************
  * @file           : PROJECT_MACRO.h
  * @brief          : 一些工程中的宏
  *
  * Copyright (c) 2026 Joe Zheng, XHU-RRC and 阿加犀(APLUX).
  *
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
***/
#ifndef __PROJECT_MACRO_H
#define __PROJECT_MACRO_H

/* ------------------------------------------------------- */
#define USER_alpha                                  0U
#define USER_1                                      1U
#define USER_2                                      2U
#define USER_other                                  3U
/* ------------------------------------------------------- */
#define __CHASSIS_X_VELOCITY_ABS_MAX__              1.8f
#define __CHASSIS_Y_VELOCITY_ABS_MAX__              1.8f
#define __CHASSIS_Z_PALSTANCE_ABS_MAX__             1.04719755f
#define __CHASSIS_M3508_RAD_P_S_MAX__               39.0f
/* ------------------------------------------------------- */

#define __PROJECT_NO_ERR                            0U
#define __ERR_CHASSIS_INNER_PID                     10U
#define __ERR_CHASSIS_OUTER_PID                     11U
#define __ERR_EC_USB_FRAME_ID                       20U
#define __ERR_EC_USB_CRC_CHECK                      21U
#define __ERR_RC_KEY_FLAG_RESET                     30U

/* ------------------------------------------------------- */
#define __EC_DEBUG_TEST_CODE                        1U

#endif
