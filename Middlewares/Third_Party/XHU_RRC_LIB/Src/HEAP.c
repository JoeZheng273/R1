/**
  **********************************************************************************
	  Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
	Permission is hereby granted, free of charge, to any person obtaining a copy of
	this software and associated documentation files (the "Software"), to deal in
	the Software without restriction, including without limitation the rights to
	use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
	the Software, and to permit persons to whom the Software is furnished to do so,
	subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
	FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
	COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
	IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
	CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

	* {
	*   This file is modified from FreeRTOS's heap_4.c ( MIT licensed ), 
	*   and is used in XHU_RRC_LIB when libUSE_FREERTOS is not defined.
	*  }
  **********************************************************************************
  */
  
#include "HEAP.h"

#if (!(libUSE_FREERTOS))

/* 生成堆. */
static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

/* 定义内存块链表节点. */
typedef struct A_BLOCK_NODE
{
	struct A_BLOCK_NODE *pxNextFreeBlock;
	size_t xBlockSize;
} BlockLink_t;

static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

static void prvHeapInit( void );

static const size_t xHeapStructSize	= 
( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

static BlockLink_t xStart, *pxEnd = NULL;

static size_t xFreeBytesRemaining = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;
static size_t xNumberOfSuccessfulAllocations = 0;
static size_t xNumberOfSuccessfulFrees = 0;

static size_t xBlockAllocatedBit = 0;

static void *pvPortMalloc( size_t xWantedSize )
{
  BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
  void *pvReturn = NULL;
  if( pxEnd == NULL )
  {
    prvHeapInit();
  }
  if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
  {
    if( xWantedSize > 0 )
    {
      xWantedSize += xHeapStructSize;
      if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
      {
        xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
      }
    }
    if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
    {
      pxPreviousBlock = &xStart;
      pxBlock = xStart.pxNextFreeBlock;
      while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
      {
        pxPreviousBlock = pxBlock;
        pxBlock = pxBlock->pxNextFreeBlock;
      }
      if( pxBlock != pxEnd )
      {
        pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );
        pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;
        if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
        {
          pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
          pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
          pxBlock->xBlockSize = xWantedSize;
          prvInsertBlockIntoFreeList( pxNewBlockLink );
        }
        xFreeBytesRemaining -= pxBlock->xBlockSize;
        if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
        {
          xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
        }
        pxBlock->xBlockSize |= xBlockAllocatedBit;
        pxBlock->pxNextFreeBlock = NULL;
        xNumberOfSuccessfulAllocations++;
      }
    }
  }
	return pvReturn;
}
static void vPortFree( void *pv )
{
  uint8_t *puc = ( uint8_t * ) pv;
  BlockLink_t *pxLink;
	if( pv != NULL )
	{
		puc -= xHeapStructSize;
		pxLink = ( void * ) puc;
		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				pxLink->xBlockSize &= ~xBlockAllocatedBit;
					xFreeBytesRemaining += pxLink->xBlockSize;
					prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
					xNumberOfSuccessfulFrees++;
			}
		}
	}
}
/*-----------------------------------------------------------*/
static size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/
static size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/
static void prvHeapInit( void )
{
  BlockLink_t *pxFirstFreeBlock;
  uint8_t *pucAlignedHeap;
  size_t uxAddress;
  size_t xTotalHeapSize = configTOTAL_HEAP_SIZE;
	uxAddress = ( size_t ) ucHeap;
	if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
	{
		uxAddress += ( portBYTE_ALIGNMENT - 1 );
		uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
		xTotalHeapSize -= uxAddress - ( size_t ) ucHeap;
	}
	pucAlignedHeap = ( uint8_t * ) uxAddress;
	xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	xStart.xBlockSize = ( size_t ) 0;
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= xHeapStructSize;
	uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
	pxEnd = ( void * ) uxAddress;
	pxEnd->xBlockSize = 0;
	pxEnd->pxNextFreeBlock = NULL;
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = pxEnd;
	xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
  BlockLink_t *pxIterator;
  uint8_t *puc;
	for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != pxEnd )
		{
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
}
/*-----------------------------------------------------------*/
static void vPortGetHeapStats( HeapStats_t *pxHeapStats )
{
  BlockLink_t *pxBlock;
  size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY;
  pxBlock = xStart.pxNextFreeBlock;
  if( pxBlock != NULL )
  {
    do
    {
      xBlocks++;
      if( pxBlock->xBlockSize > xMaxSize )
      {
        xMaxSize = pxBlock->xBlockSize;
      }
      if( pxBlock->xBlockSize < xMinSize )
      {
        xMinSize = pxBlock->xBlockSize;
      }
      pxBlock = pxBlock->pxNextFreeBlock;
    } while( pxBlock != pxEnd );
  }
	pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
	pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
	pxHeapStats->xNumberOfFreeBlocks = xBlocks;
	taskENTER_CRITICAL();
	{
		pxHeapStats->xAvailableHeapSpaceInBytes = xFreeBytesRemaining;
		pxHeapStats->xNumberOfSuccessfulAllocations = xNumberOfSuccessfulAllocations;
		pxHeapStats->xNumberOfSuccessfulFrees = xNumberOfSuccessfulFrees;
		pxHeapStats->xMinimumEverFreeBytesRemaining = xMinimumEverFreeBytesRemaining;
	}
	taskEXIT_CRITICAL();
}
#else
extern void *pvPortMalloc(size_t xSize);
extern void vPortFree(void *pv);
extern size_t xPortGetFreeHeapSize(void);
extern size_t xPortGetMinimumEverFreeHeapSize(void);
extern void vPortGetHeapStats(HeapStats_t *pxHeapStats);
#endif

void* MALLOC(size_t Size)
{
  return pvPortMalloc(Size);
}

void Free_Only(void *p)
{
  vPortFree(p);
}

size_t GetFreeHeapSize(void)
{
  return xPortGetFreeHeapSize();
}

size_t GetMinimumEverFreeHeapSize(void)
{
  return xPortGetMinimumEverFreeHeapSize();
}

void GetHeapStats(HeapStats_t *pHeapStats)
{
  if(pHeapStats != NULL)
  {
    vPortGetHeapStats(pHeapStats);
  }
}
