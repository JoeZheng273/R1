#include "bsp_led.h"
#include "gpio.h"

#define bspLED_8_Pin                           GPIO_PIN_8
#define bspLED_8_GPIO_Port                     GPIOG
#define bspLED_7_Pin                           GPIO_PIN_7
#define bspLED_7_GPIO_Port                     GPIOG
#define bspLED_6_Pin                           GPIO_PIN_6
#define bspLED_6_GPIO_Port                     GPIOG
#define bspLED_5_Pin                           GPIO_PIN_5
#define bspLED_5_GPIO_Port                     GPIOG
#define bspLED_4_Pin                           GPIO_PIN_4
#define bspLED_4_GPIO_Port                     GPIOG
#define bspLED_3_Pin                           GPIO_PIN_3
#define bspLED_3_GPIO_Port                     GPIOG
#define bspLED_2_Pin                           GPIO_PIN_2
#define bspLED_2_GPIO_Port                     GPIOG
#define bspLED_1_Pin                           GPIO_PIN_1
#define bspLED_1_GPIO_Port                     GPIOG
#define bspLED_RED_Pin                         GPIO_PIN_11
#define bspLED_RED_GPIO_Port                   GPIOE
#define bspLED_GREEN_Pin                       GPIO_PIN_14
#define bspLED_GREEN_GPIO_Port                 GPIOF

void bsp_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, bspLED_8_Pin|bspLED_7_Pin|bspLED_6_Pin|bspLED_5_Pin
                          |bspLED_4_Pin|bspLED_3_Pin|bspLED_2_Pin|bspLED_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(bspLED_RED_GPIO_Port, bspLED_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(bspLED_GREEN_GPIO_Port, bspLED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_8_Pin LED_7_Pin LED_6_Pin LED_5_Pin
                           LED_4_Pin LED_3_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = bspLED_8_Pin|bspLED_7_Pin|bspLED_6_Pin|bspLED_5_Pin
                          |bspLED_4_Pin|bspLED_3_Pin|bspLED_2_Pin|bspLED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = bspLED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(bspLED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = bspLED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(bspLED_GREEN_GPIO_Port, &GPIO_InitStruct);
}

void bsp_LED_ALL_ON(void)
{
  HAL_GPIO_WritePin(GPIOG, bspLED_8_Pin|bspLED_7_Pin|bspLED_6_Pin|bspLED_5_Pin
      |bspLED_4_Pin|bspLED_3_Pin|bspLED_2_Pin|bspLED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(bspLED_RED_GPIO_Port, bspLED_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(bspLED_GREEN_GPIO_Port, bspLED_GREEN_Pin, GPIO_PIN_RESET);
}

void bsp_LED_ALL_OFF(void)
{
  HAL_GPIO_WritePin(GPIOG, bspLED_8_Pin|bspLED_7_Pin|bspLED_6_Pin|bspLED_5_Pin
      |bspLED_4_Pin|bspLED_3_Pin|bspLED_2_Pin|bspLED_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(bspLED_RED_GPIO_Port, bspLED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(bspLED_GREEN_GPIO_Port, bspLED_GREEN_Pin, GPIO_PIN_SET);
}

void bsp_LED_x_ON(uint16_t LED_x)
{
  HAL_GPIO_WritePin(GPIOG, LED_x, GPIO_PIN_RESET);
}

void bsp_LED_x_OFF(uint16_t LED_x)
{
  HAL_GPIO_WritePin(GPIOG, LED_x, GPIO_PIN_SET);
}

void bsp_LED_GREEN_ON(void)
{
  HAL_GPIO_WritePin(bspLED_GREEN_GPIO_Port, bspLED_GREEN_Pin, GPIO_PIN_RESET);
}

void bsp_LED_GREEN_OFF(void)
{
  HAL_GPIO_WritePin(bspLED_GREEN_GPIO_Port, bspLED_GREEN_Pin, GPIO_PIN_SET);
}

void bsp_LED_RED_ON(void)
{
  HAL_GPIO_WritePin(bspLED_RED_GPIO_Port, bspLED_RED_Pin, GPIO_PIN_RESET);
}

void bsp_LED_RED_OFF(void)
{
  HAL_GPIO_WritePin(bspLED_RED_GPIO_Port, bspLED_RED_Pin, GPIO_PIN_SET);
}
