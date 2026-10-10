#ifndef __MODCRITICAL_SECTION_CONF_H
#define __MODCRITICAL_SECTION_CONF_H

#include "libXHU_RRC_LIB_Conf.h"

#if (!(libUSE_FREERTOS))
// 此处应该包含所使用的芯片的总头文件，例如STM32F4系列，就 #include "stm32f4xx.h"。
#include "stm32f4xx.h"
#define configCRITICAL_NEST_DEPTH_MAX           16
#endif

#endif
