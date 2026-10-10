#include "Critical_Section.h"
#include "mod_Critical_Section_Conf.h"
#include "stdint.h"

#if (!(libUSE_FREERTOS))
// 此处应该包含所使用的芯片的总头文件，例如STM32F4系列，就 #include "stm32f4xx.h"。
#include "stm32f4xx.h"
/* -------------------------------以下无需修改------------------------------- */
static uint32_t primask[configCRITICAL_NEST_DEPTH_MAX] = {0};
static uint32_t Counter = 0;
#else
#include "FreeRTOS.h"
#include "task.h"
#endif

void Critical_Enter(void)
{
#if (!(libUSE_FREERTOS))
if(Counter < configCRITICAL_NEST_DEPTH_MAX)
{
  primask[Counter] = __get_PRIMASK();
  __disable_irq();
  Counter++;
}
#else
taskENTER_CRITICAL();
#endif
}

void Critical_Exit(void)
{
#if (!(libUSE_FREERTOS))
if(Counter > 0)
{
  Counter--;
  __set_PRIMASK(primask[Counter]);
}
#else
taskEXIT_CRITICAL();
#endif
}
