/**
  **********************************************************************************
  * @file    xhu_rrc_lib_core.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  *
  * @brief   XHU_RRC_LIB 的内部头文件.
  *
  **********************************************************************************
  * @attention
  * 使用 XHU_RRC_LIB 必须先定义宏  XHU_RRC_LIB_ENABLE.
  * 我将每个分文件都尽量让其可以独立使用.
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
  */

#ifndef __XHU_RRC_LIB_CORE_H
#define __XHU_RRC_LIB_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined (XHU_RRC_LIB_ENABLE)
//编译器指令.
  #ifndef __weak
    #define __weak  __attribute__((weak))
  #endif
  #include "stdio.h"
  #include "stddef.h"
  #include "stdbool.h"
  #include "stdint.h"
  #include "libXHU_RRC_LIB_Conf.h"

#endif

#ifdef __cplusplus
}
#endif

#endif
