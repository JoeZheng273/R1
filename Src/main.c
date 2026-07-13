/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_assistant.h"
#include "bsp_led.h"
#include "XHU_RRC_LIB.h"
#include "chassis.h"
#include "remote_ctrl.h"
#include "platform.h"
#include "safe_task.h"
#include "can_receive.h"
#include "robot_arm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile _Bool HAL_Inited_Flag = 0;
#if Serial_Plot_Enable
uint8_t channelname[] = "vx,vy,wz";
float PlotData[3] = {0};
const Serial_Plot_Packet_t Packet = {
  .huart = &huart6,
  .name = channelname,
  .name_len = sizeof(channelname),
  .plot_data = PlotData,
  .data_len = sizeof(PlotData),
  .timeout_ms = 50
};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Robot_Init(void)
{
  /* ---------- BSP Init ---------- */
  bsp_LED_Init();
  bsp_LED_ALL_OFF();
  HAL_Inited_Flag = 1;
  RC_TurnOff();
  HAL_Delay(10);
  RC_TurnOn();
  HAL_Delay(10);
  can_filter_init(&hcan1);
  can_filter_init(&hcan2);
  bsp_LED_GREEN_ON();
  /* -------- SafeTask Start ------- */
  HAL_TIM_Base_Start_IT(&htim7);
  Safe_task_Update_LimFlag();
  /* --------- Module Init --------- */
  Chassis_Init();
  RC_Init();
  RobotArm_Init();
  PlatForm_Init();
  /* ----- PID Controler Start ----- */
  HAL_TIM_Base_Start_IT(&htim6);
  /* ------------ Ready ------------ */
  Safe_task_Back_to_zero();
  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART7_Init();
  MX_CRC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  Robot_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(RC_GetRxCpltFlag())
    {
      RC_ProcessedData();
    }
    if(RC_GetKeyUpdateFlag())
    {
      RC_KeyProcessed();
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(CAN_Receive_Error_GetFlag())
    {
      CAN_Receive_ErrorProcessed();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)  //PID 2ms
  {
    Chassis_TIM_PeriodCallback();
    PlatForm_TIM_PeriodCallback();
  }
  if(htim->Instance == TIM7)  // SafeTask
  {
    Safe_task_TIM_PeriodCallback();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  uint32_t i = 0;
  uint32_t cycle = SystemCoreClock/15;
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    if(HAL_Inited_Flag)
    {
      bsp_LED_RED_ON();
      for(i=0;i<cycle;i++)
      {   }
      bsp_LED_RED_OFF();
      for(i=0;i<cycle;i++)
      {   }
    }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
