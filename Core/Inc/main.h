/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M0_DRV_CS_Pin GPIO_PIN_13
#define M0_DRV_CS_GPIO_Port GPIOC
#define M1_DRV_CS_Pin GPIO_PIN_14
#define M1_DRV_CS_GPIO_Port GPIOC
#define M0_SO1_Pin GPIO_PIN_0
#define M0_SO1_GPIO_Port GPIOC
#define M0_SO2_Pin GPIO_PIN_1
#define M0_SO2_GPIO_Port GPIOC
#define M1_SO2_Pin GPIO_PIN_2
#define M1_SO2_GPIO_Port GPIOC
#define M1_SO1_Pin GPIO_PIN_3
#define M1_SO1_GPIO_Port GPIOC
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define M1_TEMP_Pin GPIO_PIN_4
#define M1_TEMP_GPIO_Port GPIOA
#define AUX_TEMP_Pin GPIO_PIN_5
#define AUX_TEMP_GPIO_Port GPIOA
#define VBUS_S_Pin GPIO_PIN_6
#define VBUS_S_GPIO_Port GPIOA
#define M0_TEMP_Pin GPIO_PIN_5
#define M0_TEMP_GPIO_Port GPIOC
#define AUX_L_Pin GPIO_PIN_10
#define AUX_L_GPIO_Port GPIOB
#define AUX_H_Pin GPIO_PIN_11
#define AUX_H_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define nFAULT_Pin GPIO_PIN_2
#define nFAULT_GPIO_Port GPIOD
#define nFAULT_EXTI_IRQn EXTI2_IRQn
#define M0_ENC_CS_Pin GPIO_PIN_6
#define M0_ENC_CS_GPIO_Port GPIOB
#define M1_ENC_CS_Pin GPIO_PIN_7
#define M1_ENC_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
