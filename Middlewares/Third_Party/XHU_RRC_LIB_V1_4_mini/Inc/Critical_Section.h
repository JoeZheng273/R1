/**
  **********************************************************************************
  * @file    Critical_Section.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  *
  * @brief   XHU_RRC_LIB 的 临界保护 部分.
  *
  **********************************************************************************
  * @attention
  * 本人能力有限,请见谅,如有BUG或其他问题,希望你能帮忙改进一下,十分感谢.
  * 欢迎继续扩充,扩充或修改后,欢迎留下名字当作纪念.期待你的加入.
  *
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

#ifndef __CRITICAL_SECTION_H
#define __CRITICAL_SECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  *@brief   进入临界区,开启临界区保护(禁用中断).
  *@param   None.
  *@retval  None.
  **/
void Critical_Enter(void);

/**
  *@brief   离开临界区,关闭临界区保护(恢复中断).
  *@param   None.
  *@retval  None.
  **/
void Critical_Exit(void);

#ifdef __cplusplus
}
#endif

#endif
