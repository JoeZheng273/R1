#ifndef __MODCRITICAL_SECTION_CONF_H
#define __MODCRITICAL_SECTION_CONF_H

#include "libXHU_RRC_LIB_Conf.h"

#if (!(libUSE_FREERTOS))
#define configCRITICAL_NEST_DEPTH_MAX           16
#endif

#endif
