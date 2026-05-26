/**
  **********************************************************************************
  * @file    modXHU_RRC_LIB_HEAP_Conf.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  * @brief   XHU_RRC_LIB 中的内存管理配置文件.
  *
  **********************************************************************************
  * @attention
  * 本人能力有限,请见谅,如有BUG或其他问题,希望你能帮忙改进一下,十分感谢.
  * 欢迎继续扩充.扩充或修改后,欢迎留下名字当作纪念.期待你的加入.
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
  */

#ifndef __MODXHU_RRC_LIB_HEAP_CONF_H
#define __MODXHU_RRC_LIB_HEAP_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include "XHU_RRC_LIB_HEAP_Def.h"

#if (!(libUSE_FREERTOS))

/* 定义堆区的大小. unit: Byte. */
#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE                10240
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif
