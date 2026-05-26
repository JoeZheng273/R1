#ifndef __MOTOR_H
#define __MOTOR_H

#include "stdint.h"

typedef enum    //  如果有更多CAN总线可以自行添加.
{
  can_bus_1 = 0,
  can_bus_2 = 1
} can_bus;

typedef enum    //  如果有更多电机ID可以自行添加.
{
  ID_1 = 0,
  ID_2 = 1,
  ID_3 = 2,
  ID_4 = 3,
  ID_5 = 4,
  ID_6 = 5,
  ID_7 = 6,
  ID_8 = 7
} ESC_ID;

/**
  *@brief 用于存放电机情况的过程变量(实际值).
  */
typedef struct
{
	float Angle;        /*!<  输出轴转过的角度.              */
	float Omega;        /*!<  输出轴的角速度.                */
  float Omega_ecd;    /*!<  输出轴的角速度,由编码器测得.   */
  float Torque;       /*!<  输出轴的扭矩.                  */
  float Current;      /*!<  实际的电流.                    */
  float Temperate;    /*!<  实际的温度.                    */
} PV_t;

#endif
