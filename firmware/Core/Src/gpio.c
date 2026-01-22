/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, debug_Pin|motor3_dir_Pin|cs_enc_3_Pin|cs_enc_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, cs_enc_2_Pin|motor2_Cs_Pin|motor2_dir_Pin|nBRAKE_Pin
                          |motor1_Cs_Pin|MCU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motor3_Cs_Pin|motor1_dir_Pin|enable_Pin|motor2_led_Pin
                          |motor3_led_Pin|motor1_led_Pin, GPIO_PIN_RESET);

  /* ========================================================== */
  /* GPIOC CONFIGURATION                                        */
  /* ========================================================== */

  // Normal pins (No Pull)
  GPIO_InitStruct.Pin = debug_Pin|cs_enc_3_Pin|cs_enc_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Motor 3 Direction (Add Pull-Down)
  GPIO_InitStruct.Pin = motor3_dir_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ========================================================== */
  /* GPIOA CONFIGURATION (Motor 2)                              */
  /* ========================================================== */

  // 1. Chip Selects and Brake (Keep NOPULL)
  // We removed motor2_dir_Pin from this list
  GPIO_InitStruct.Pin = cs_enc_2_Pin|motor2_Cs_Pin|nBRAKE_Pin
                          |motor1_Cs_Pin|MCU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 2. Motor 2 Direction (Set PULLDOWN)
  // This forces the pin to Logic 0 (Reverse/Forward) if undefined
  GPIO_InitStruct.Pin = motor2_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ========================================================== */
  /* GPIOB CONFIGURATION (Motor 1)                              */
  /* ========================================================== */

  // Normal pins
  GPIO_InitStruct.Pin = motor3_Cs_Pin|enable_Pin|motor2_led_Pin
                          |motor3_led_Pin|motor1_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Motor 1 Direction (Add Pull-Down)
  GPIO_InitStruct.Pin = motor1_dir_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
