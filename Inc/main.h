/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define BOT_A_Pin GPIO_PIN_5
#define BOT_A_GPIO_Port GPIOA
#define BOT_A_EXTI_IRQn EXTI9_5_IRQn
#define BOT_B_Pin GPIO_PIN_6
#define BOT_B_GPIO_Port GPIOA
#define BOT_B_EXTI_IRQn EXTI9_5_IRQn
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOF
#define LED_TEMP_Pin GPIO_PIN_13
#define LED_TEMP_GPIO_Port GPIOF
#define LED_CRONO_Pin GPIO_PIN_9
#define LED_CRONO_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD
#define SWITCH_ON_OFF_Pin GPIO_PIN_8
#define SWITCH_ON_OFF_GPIO_Port GPIOB
#define SWITCH_ON_OFF_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
