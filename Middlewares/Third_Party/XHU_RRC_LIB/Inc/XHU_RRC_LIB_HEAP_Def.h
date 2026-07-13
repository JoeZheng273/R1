/**
  **********************************************************************************
  * @file    XHU_RRC_LIB_HEAP_def.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  * @brief   XHU_RRC_LIB 中的内存管理的宏定义.
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

#ifndef __XHU_RRC_LIB_HEAP_DEF_H
#define __XHU_RRC_LIB_HEAP_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xhu_rrc_lib_core.h"
#include "Critical_Section.h"

#if (!(libUSE_FREERTOS))

#define portMAX_DELAY ( uint32_t ) 0xffffffffUL

#ifndef taskENTER_CRITICAL
#define taskENTER_CRITICAL()          Critical_Enter()
#endif

#ifndef taskEXIT_CRITICAL
#define taskEXIT_CRITICAL()           Critical_Exit()
#endif

#define configSUPPORT_DYNAMIC_ALLOCATION     1

#define portBYTE_ALIGNMENT			             8

#if portBYTE_ALIGNMENT == 8
	#define portBYTE_ALIGNMENT_MASK ( 0x0007 )
#endif

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

#endif

#ifdef __cplusplus
}
#endif

#endif
