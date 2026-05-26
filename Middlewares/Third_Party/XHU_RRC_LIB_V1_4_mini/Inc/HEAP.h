/**
  **********************************************************************************
  * @file    HEAP.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  * @brief   XHU_RRC_LIB 中的内存管理.
  *
  **********************************************************************************
  * @attention
  * {
  *   本模块调用了FreeRTOS的内存管理算法,并且在libUSE_FREERTOS未定义时,
  *	  提供了一个独立的内存管理实现, 以供不使用FreeRTOS的用户使用, 
  *	  该内存管理算法是基于FreeRTOS的heap_4.c修改而来, 以适应XHU_RRC_LIB的需求.
  *  }
  * 
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

#ifndef __HEAP_H
#define __HEAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mod_HEAP_Conf.h"

#if (!(libUSE_FREERTOS))
typedef struct xHeapStats
{
	size_t xAvailableHeapSpaceInBytes;
	size_t xSizeOfLargestFreeBlockInBytes;
	size_t xSizeOfSmallestFreeBlockInBytes;
	size_t xNumberOfFreeBlocks;
	size_t xMinimumEverFreeBytesRemaining;
	size_t xNumberOfSuccessfulAllocations;
	size_t xNumberOfSuccessfulFrees;
} HeapStats_t;
#else
#include "FreeRTOS.h"
#endif

/*------------- HEAP API --------------------*/
void* MALLOC(size_t Size);

size_t GetFreeHeapSize(void);

size_t GetMinimumEverFreeHeapSize(void);

void GetHeapStats(HeapStats_t *pHeapStats);

/**
  *@brief   仅对内存进行释放,不将指针变量置空.
  **/
void Free_Only(void *p);

#define FREE(ptr)                                   \
        do {                                        \
          extern void Free_Only(void *p);           \
          Free_Only((ptr));                         \
          (ptr) = NULL;                             \
        } while(0)

/*--------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
