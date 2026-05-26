#include "can_receive.h"
#include "XHU_RRC_LIB.h"
#include "can.h"
#include "robot_arm.h"

void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st = {0};
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.SlaveStartFilterBank = 14;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
  can_filter_st.FilterBank = 14;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0200;
  can_filter_st.FilterMaskIdLow =  0x0200;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

uint32_t Chassis_Send_CAN_Cmd(int16_t motor1, int16_t motor2, 
                                        int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box = 0;
  CAN_TxHeaderTypeDef  tx_message = {0};
  uint8_t can_send_data[8]={0};
  tx_message.StdId = 0x200;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  can_send_data[0] = motor1 >> 8;
  can_send_data[1] = motor1;
  can_send_data[2] = motor2 >> 8;
  can_send_data[3] = motor2;
  can_send_data[4] = motor3 >> 8;
  can_send_data[5] = motor3;
  can_send_data[6] = motor4 >> 8;
  can_send_data[7] = motor4;
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
  return send_mail_box;
}

uint32_t Plat_Send_CAN_Cmd(int16_t motor1, int16_t motor2, 
                                        int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box = 0;
  CAN_TxHeaderTypeDef  tx_message = {0};
  uint8_t can_send_data[8]={0};
  tx_message.StdId = 0x1FF;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  can_send_data[0] = motor1 >> 8;
  can_send_data[1] = motor1;
  can_send_data[2] = motor2 >> 8;
  can_send_data[3] = motor2;
  can_send_data[4] = motor3 >> 8;
  can_send_data[5] = motor3;
  can_send_data[6] = motor4 >> 8;
  can_send_data[7] = motor4;
  HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
  return send_mail_box;
}

uint32_t RobotArm_Send_CAN_Cmd(uint32_t CAN_ID, const uint8_t *pData)
{
  uint32_t send_mail_box = 0;
  CAN_TxHeaderTypeDef  tx_message = {0};
  tx_message.StdId = CAN_ID;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  HAL_CAN_AddTxMessage(&hcan2, &tx_message, pData, &send_mail_box);
  return send_mail_box;
}

/**
  *@brief   HAL Callback
  **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header={0};
  uint8_t rx_data[8]={0};
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    uint8_t index = rx_header.StdId - 0x201;
    djiMotor_CANFeedbackCallback(index,can_bus_1,rx_data);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header={0};
  uint8_t rx_data[8]={0};
  if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    if(rx_header.StdId == 0x121)
    {
      RobotArm_Clamp_Processed(rx_data);
    }
    if(rx_header.StdId == 0x122)
    {
      RobotArm_Arm_Processed(rx_data);
    }
  }
}
