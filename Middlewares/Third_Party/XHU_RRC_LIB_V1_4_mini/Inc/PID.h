/**
  **********************************************************************************
  * @file    PID.h
  * @author  XHU_RRC实验室软件组  24届 郑琪川
  *
  *
  * @brief   XHU_RRC_LIB 的PID控制算法部分.
  *
  **********************************************************************************
  * @attention
  * 本人能力有限,请见谅,如有BUG或其他问题,希望你能帮忙改进一下,十分感谢.
  * 欢迎继续扩充.扩充或修改后,欢迎留下名字当作纪念.期待你的加入.
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
  **/
#ifndef  __PID_H
#define  __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "math.h"
#include "HEAP.h"

/**
  *@addtogroup    XHU_RRC_LIB
  *@{
  **/

/**@defgroup  PID_BASIC_TYPE_DEFINE    PID Basic type define
  *@brief   PID基本数据类型定义.
  *@{
  **/
typedef unsigned int PID_Status;  /*!<  @ref PID_Error_Code   */
typedef struct PID_InstanceTypeDef PID_Inst_t;  /* PID结构体的不完全类型声明. */
/* typedef指针类型会使指针类型成为一个整体,需注意const的修饰对象. */
typedef PID_Inst_t* PID_t;

/**@brief   Forward_function 前馈回调函数指针类型.
  *@note    第1个参数是 Set Point,
  *@note    第2个参数是 调用此回调函数的PID控制器实例指针.
  */
typedef float(*Forward_func_t)(const float Target, const PID_Inst_t* const This);

/**@brief   SumDecay_function 积分衰减回调函数指针类型.
  *@note    参数为 调用此回调函数的PID控制器实例指针.
  */
typedef float(*SumDecay_func_t)(const PID_Inst_t* const This);

typedef struct
{
	float Kf;                       /*!<  前馈项系数            */
	float Kp;                       /*!<  比例项系数            */
	float Ki;                       /*!<  积分项系数            */
	float Kd;                       /*!<  微分项系数            */
  float Alpha;                    /*!<  一阶滤波的系数        */
  float Epsilon;                  /*!<  误差死区              */
	float Summax;                   /*!<  积分限幅              */
	float Outputmax;                /*!<  输出限幅              */
	float SumOp_region;             /*!<  积分有效区间          */
} PID_Basic_Param_t; /*!< 参数结构体,是PID控制器必需的参数    */

typedef struct
{
	float Sum;                      /*!<  积分求和              */
	float last_Err;                 /*!<  上次误差              */
	float Err;                      /*!<  本次误差              */
	float Change;                   /*!<  两次误差之间的改变量  */
  float Change_filt;              /*!<  一阶滤波后的Change    */
	float Output_f;                 /*!<  原始计算得到的输出值  */
	float F_term;                   /*!<  前馈项                */
	float P_term;                   /*!<  比例项                */
	float I_term;                   /*!<  积分项                */
	float D_term;                   /*!<  微分项                */
} PID_Container_t; /*!< 容器结构体,PID实例中存在这些中间变量  */

/**
  *@} PID Basic type define
  **/

/**@defgroup  PID_ERROR_CODE    PID error code
  *@brief   PID控制算法的返回值,PID运行状态.
  *@{
  **/
#define   PID_WARN          0x00000010U   /*!< PID相关操作所得结果不实时或可能有误.  */
#define   PID_BUSY          0x00000008U   /*!< PID实例互斥锁已锁,无法修改.           */
#define   PID_WARN_3        PID_BUSY      /*!< PID实例互斥锁已锁,访问失败.           */
#define   PID_WARN_2        0x00000004U   /*!< PID实例积分衰减为空.                  */
#define   PID_WARN_1        0x00000002U   /*!< PID实例前馈回调为空.                  */
#define   PID_WARN_0        0x00000001U   /*!< PID实例滤波系数无效.                  */
#define   PID_OK            0x00000000U   /*!< PID相关操作运行正常.                  */
#define   PID_ERR_0         0xFFFFFFFFU   /*!< PID相关操作的参数列表有误.            */
#define   PID_ERR_1         0xFFFFFFFEU   /*!< PID实例未对参数赋值或未使能.          */

/**
  *@} PID error code
  **/

/**@defgroup  PID_API        PID API
  *@brief   PID控制算法的应用程序接口.
  *@{
  **/

/* ---------------- PID  API  START ----------------- */

/**@brief   创建PID控制器实例.
  *@param   None.
  *@retval  PID控制器实例指针.
  **/
PID_t New_PID_Instance(void);

/**@brief   PID控制算法.
  *@param   SP(Set Point)           目标值.
  *@param   PV(Process Variable)    过程变量(实际值).
  *@param   pPID_Instance           PID控制器实例.
  *@param   pCO(Controller Output)  (可选)整型输出值.
  *@retval  PID_Status 类型,代表PID的运行状态. @ref  PID_Error_Code .
  *@note    全量式PID(位置式PID),如需整型输出,则在pCO填入接收输出的
  *@note    变量的指针,如果PID运行成功,就会将整型输出返回pCO所指向的
  *@note    整型变量中. 如不需要整形输出,则pCO填入NULL.
  **/
PID_Status PIDf_Control(float SP, float PV, PID_t pPID_Instance, int *pCO);

/**
  *@brief   获取浮点型CO(Controller Output)
  *@param   pPID_Instance           PID控制器实例.
  *@param   pResult                 用于存放PID浮点型输出的变量的指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_GetOutput_f(const PID_Inst_t *pPID_Instance, float *pResult);

/**@brief   删除PID控制器实例,并将指针变量置为NULL.
  *@param   PID控制器实例的指针.
  *@retval  None.
  *@note    必须传入变量,否则置空会出错,如果传入常量应调用 Free_Only .
  **/
#define Delete_PID_Instance(ptr)               FREE(ptr)

/**@brief   将PID控制器的参数置为默认值同时将自动存放的中间变量清空.
  *@param   PID控制器实例指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_InstanceSafeReset(PID_Inst_t *pPID_Instance);

/**
  *@brief   PID_InstanceSafeReset 的强制执行版本.
  *@note    随意使用可能会造成安全隐患.
  **/
void PID_InstanceReset(PID_Inst_t *pPID_Instance);

/**@brief   批量设置PID控制器的基础参数.
  *@param   PID控制器实例指针.
  *@param   已填入基础参数的结构体的指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_SetBasicParam(PID_Inst_t *pPID_Instance, const PID_Basic_Param_t *pParam_t);

/**@brief   将PID控制器的基础参数重置为默认值.
  *@param   PID控制器实例指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_ResetBasicParam(PID_Inst_t *pPID_Instance);

/**@brief   单个设置PID控制器参数.
  *@param   PID控制器实例指针.
  *@param   设置的参数的值.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  *@note    PID_SetParam_X(pPID_Instance, x)
  *@note    X 为参数名字, x 为参数的值.
  **/
PID_Status PID_SetParam_Alpha(PID_Inst_t *pPID_Instance, const float Alpha);
PID_Status PID_SetParam_Epsilon(PID_Inst_t *pPID_Instance, const float Epsilon);
PID_Status PID_SetParam_Kd(PID_Inst_t *pPID_Instance, const float Kd);
PID_Status PID_SetParam_Kf(PID_Inst_t *pPID_Instance, const float Kf);
PID_Status PID_SetParam_Ki(PID_Inst_t *pPID_Instance, const float Ki);
PID_Status PID_SetParam_Kp(PID_Inst_t *pPID_Instance, const float Kp);
PID_Status PID_SetParam_Outputmax(PID_Inst_t *pPID_Instance, const float Outputmax);
PID_Status PID_SetParam_Summax(PID_Inst_t *pPID_Instance, const float Summax);
PID_Status PID_SetParam_SumOp_region(PID_Inst_t *pPID_Instance, const float SumOp_region);

/* ----------------- 以下是非必需参数 ----------------- */
/**
  *@note    dt默认值为1,因为可以将dt整合在参数Ki和Kd中.
  *@note    dt  采样时间, unit: s.
  **/
PID_Status PID_SetParam_dt(PID_Inst_t *pPID_Instance, const float dt);

PID_Status PID_ResetParam_dt(PID_Inst_t *pPID_Instance);

/**@note    SumDecay  自定义积分衰减回调函数.
  *@note    当误差超过积分有效区间时调用. 默认清空积分.
  *@note    该回调函数的返回值在限幅后作用于积分项.
  *@note    设该回调函数返回值为 A, 则积分项 I = Ki * A * dt.
  *@note    该回调函数的参数列表规范  参考  SumDecay_function 
  **/
PID_Status PID_SetParam_SumDecay(PID_Inst_t *pPID_Instance, SumDecay_func_t SumDecay);

PID_Status PID_ResetParam_SumDecay(PID_Inst_t *pPID_Instance);

/**@note    Forward   自定义前馈回调函数.
  *@note    默认为0.
  *@note    该回调函数的返回值将直接作用于前馈项.
  *@note    设该回调函数返回值为 A, 则前馈项 F = Kf * A.
  *@note    该回调函数的参数列表规范  参考  Forward_function 
  **/
PID_Status PID_SetParam_Forward(PID_Inst_t *pPID_Instance, Forward_func_t Forward);

PID_Status PID_ResetParam_Forward(PID_Inst_t *pPID_Instance);

/* --------------- 以上为非必需参数 ---------------- */

/* --------------- 控制器使能和失能 ---------------- */
/**@brief   PID控制器使能.
  *@param   PID控制器实例指针.
  *@retval  None.
  *@note    PID_ENABLE_X(pPID_Instance)
  *@note    X 为要使能的对应项.
  **/
void PID_ENABLE_ALL(PID_Inst_t *pPID_Instance);
void PID_ENABLE_P(PID_Inst_t *pPID_Instance);
void PID_ENABLE_I(PID_Inst_t *pPID_Instance);
void PID_ENABLE_D(PID_Inst_t *pPID_Instance);
void PID_ENABLE_F(PID_Inst_t *pPID_Instance);

/**@brief   PID控制器失能.
  *@param   PID控制器实例指针.
  *@retval  None.
  *@note    PID_DISABLE_X(pPID_Instance)
  *@note    X 为要失能的对应项.
  **/
void PID_DISABLE_ALL(PID_Inst_t *pPID_Instance);
void PID_DISABLE_P(PID_Inst_t *pPID_Instance);
void PID_DISABLE_I(PID_Inst_t *pPID_Instance);
void PID_DISABLE_D(PID_Inst_t *pPID_Instance);
void PID_DISABLE_F(PID_Inst_t *pPID_Instance);


/* ---------------- 获取PID控制器内的参数 ------------------- */
/**@brief   获取PID控制器的参数.
  *@param   PID控制器实例指针.
  *@param   存放要获取的参数的值的变量的指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  *@note    PID_GetParam_X(pPID_Instance)
  *@note    X 为要获取的参数.
  **/
PID_Status PID_GetParam_Alpha(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Epsilon(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Kd(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Kf(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Ki(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Kp(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Outputmax(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_Summax(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_SumOp_region(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetParam_dt(const PID_Inst_t *pPID_Instance, float *pResult);

/**
  *@brief   对比PID实例中的回调函数指针.
  *@param   PID实例指针.
  *@param   回调函数指针.
  *@retval  相同则返回true,否则返回false.
  **/
_Bool PID_CmpParam_SumDecay(const PID_Inst_t *pPID_Instance, SumDecay_func_t SumDecay);
_Bool PID_CmpParam_Forward(const PID_Inst_t *pPID_Instance, Forward_func_t Forward);

/**@brief   一次性获取所有基础参数的值.
  *@param   PID控制器实例指针.
  *@param   用于接收PID实例基础参数的结构体指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_GetBasicParam(const PID_Inst_t *pPID_Instance, PID_Basic_Param_t *pParam_t);

/**@brief   获取PID实例的中间变量.
  *@param   PID控制器实例的指针.
  *@param   存放要获取的中间变量的值的变量的指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  *@note    PID_GetContainer_X(pPID_Instance)
  *@note    X 要获取的中间变量.
  **/
PID_Status PID_GetContainer_Change(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_Change_filt(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_D_term(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_Err(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_F_term(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_I_term(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_last_Err(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_P_term(const PID_Inst_t *pPID_Instance, float *pResult);
PID_Status PID_GetContainer_Sum(const PID_Inst_t *pPID_Instance, float *pResult);

/**@brief   一次性获取所有中间变量的值.
  *@param   PID控制器实例指针.
  *@param   用于接收PID实例中间变量的结构体指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_GetContainer(const PID_Inst_t *pPID_Instance, PID_Container_t *pContainer_t);

/**
  *@brief   将PID实例的中间变量全部清空.
  *@param   PID控制器实例指针.
  *@retval  PID_Status 类型,代表PID相关操作的状态. @ref  PID_Error_Code .
  **/
PID_Status PID_ResetContainer(PID_Inst_t *pPID_Instance);

/**
  *@brief   强制解锁PID实例的互斥锁.
  *@note    随意使用可能造成安全隐患.
  **/
void PID_MutexForceUnLock(PID_Inst_t *pPID_Instance);

/* --------------------------------------------------------------- */
/**
  *@note{
  *         下面的API用于静态创建PID实例,为用户提供了不依赖于动态内存
  *         分配的实现可能,但相关安全考虑和安全处理由用户自行负责.
  *@}
  **/
/**
  *@brief   获取PID实例的字节数.
  **/
size_t PID_Sizeof_Instance(void);

/**
  *@brief{
  *         为不使用动态内存分配 New_PID_Instance 接口的实例
  *         对象提供初始化,该接口仅具有将指针 p 当作PID实例的
  *         句柄,并按照PID实例的内存分布提供与 New_PID_Instance 
  *         相同的初始化的功能.
  *@}
  **/
void PID_Instance_Init(void *p);

/* ----------------------- PID  API  END ------------------------ */

/**
  *@} PID API
  **/

/**
  *@} XHU_RRC_LIB
  **/

#ifdef __cplusplus
}
#endif

#endif
