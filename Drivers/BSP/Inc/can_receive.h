#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "stdint.h"

void can_filter_init(void);

uint32_t Chassis_Send_CAN_Cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

uint32_t Plat_Send_CAN_Cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

uint32_t RobotArm_Send_CAN_Cmd(uint32_t CAN_ID, const uint8_t *pData);

#endif
