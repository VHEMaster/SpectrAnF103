/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define LED_C9_Pin GPIO_PIN_1
#define LED_C9_GPIO_Port GPIOA
#define LED_C8_Pin GPIO_PIN_2
#define LED_C8_GPIO_Port GPIOA
#define LED_C7_Pin GPIO_PIN_3
#define LED_C7_GPIO_Port GPIOA
#define LED_C6_Pin GPIO_PIN_4
#define LED_C6_GPIO_Port GPIOA
#define LED_C5_Pin GPIO_PIN_5
#define LED_C5_GPIO_Port GPIOA
#define LED_C4_Pin GPIO_PIN_6
#define LED_C4_GPIO_Port GPIOA
#define LED_C3_Pin GPIO_PIN_7
#define LED_C3_GPIO_Port GPIOA
#define LED_C2_Pin GPIO_PIN_0
#define LED_C2_GPIO_Port GPIOB
#define LED_C1_Pin GPIO_PIN_1
#define LED_C1_GPIO_Port GPIOB
#define LED_C0_Pin GPIO_PIN_2
#define LED_C0_GPIO_Port GPIOB
#define LED_R13_Pin GPIO_PIN_10
#define LED_R13_GPIO_Port GPIOB
#define LED_R12_Pin GPIO_PIN_11
#define LED_R12_GPIO_Port GPIOB
#define LED_R11_Pin GPIO_PIN_12
#define LED_R11_GPIO_Port GPIOB
#define LED_R10_Pin GPIO_PIN_8
#define LED_R10_GPIO_Port GPIOA
#define LED_R9_Pin GPIO_PIN_11
#define LED_R9_GPIO_Port GPIOA
#define LED_R8_Pin GPIO_PIN_12
#define LED_R8_GPIO_Port GPIOA
#define LED_R7_Pin GPIO_PIN_15
#define LED_R7_GPIO_Port GPIOA
#define LED_R6_Pin GPIO_PIN_3
#define LED_R6_GPIO_Port GPIOB
#define LED_R5_Pin GPIO_PIN_4
#define LED_R5_GPIO_Port GPIOB
#define LED_R4_Pin GPIO_PIN_5
#define LED_R4_GPIO_Port GPIOB
#define LED_R3_Pin GPIO_PIN_6
#define LED_R3_GPIO_Port GPIOB
#define LED_R2_Pin GPIO_PIN_7
#define LED_R2_GPIO_Port GPIOB
#define LED_R1_Pin GPIO_PIN_8
#define LED_R1_GPIO_Port GPIOB
#define LED_R0_Pin GPIO_PIN_9
#define LED_R0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
