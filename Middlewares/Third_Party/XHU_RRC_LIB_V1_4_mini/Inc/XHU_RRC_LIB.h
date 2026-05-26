/**
  **********************************************************************************
  * @file    XHU_RRC_Lib.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  *
  * @brief   使用XHU_RRC_LIB时应包含的头文件.
  *
  **********************************************************************************
  * @attention
  * 使用 XHU_RRC_LIB 必须先定义 宏  XHU_RRC_LIB_ENABLE.
  * 使用 XHU_RRC_LIB 只需要包含这个头文件.
  * 本人能力有限,请见谅,如有BUG或其他问题,希望你能帮忙改进一下,十分感谢.
  * 欢迎继续扩充.扩充或修改后,欢迎留下名字当作纪念.期待你的加入.
  *
  **********************************************************************************
  * @history
  *  Version        Date                Author              Modification
  *  V1.0.0         2025-12-21          郑琪川              
  *
  *
  *
  *
  **********************************************************************************
  *
  **********************************************************************************
  */

#ifndef __XHU_RRC_LIB_H
#define __XHU_RRC_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined (XHU_RRC_LIB_ENABLE)  /*!< 使用 XHU_RRC_LIB 必须定义 XHU_RRC_LIB_ENABLE   */
  #include "libXHU_RRC_LIB_Conf.h"
  #include "HEAP.h"
  #include "PID.h"
  #if (libUSE_DJIMOTOR)
    #include "DJI_Motor.h"
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif
