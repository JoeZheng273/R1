/**
  **********************************************************************************
  * @file    libXHU_RRC_LIB_Conf.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  * @brief   XHU_RRC_LIB 配置文件.
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

#ifndef __LIBXHU_RRC_LIB_CONF_H
#define __LIBXHU_RRC_LIB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/**@defgroup    XHU_RRC_LIB   XHU_RRC_LIB  
  *@{
  */

/**@defgroup    XHU_RRC_LIB_CONFIGURATION_BASE   XHU_RRC_LIB Configuration Base
  *@{
  */
  
#define   DISABLE_lib_               0
#define   ENABLE_lib_               (!(DISABLE_lib_))

/**@} XHU_RRC_LIB_CONFIGURATION_BASE
  */

/**@defgroup    XHU_RRC_LIB_CONFIGURATION   XHU_RRC_LIB Configuration  
***@{
****/
/*--------------------- Configuration ----------------------*/
#define   libUSE_FREERTOS                       DISABLE_lib_
#define   libUSE_DJIMOTOR                       ENABLE_lib_




/*---------------------------------------------------------*/
/**@} XHU_RRC_LIB_CONFIGURATION
****/

/**@} XHU_RRC_LIB
  */

#ifdef __cplusplus
}
#endif

#endif
