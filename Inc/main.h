/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_Pin GPIO_PIN_15
#define SW_GPIO_Port GPIOC
#define FR_Pin GPIO_PIN_0
#define FR_GPIO_Port GPIOC
#define SR_Pin GPIO_PIN_1
#define SR_GPIO_Port GPIOC
#define SL_Pin GPIO_PIN_0
#define SL_GPIO_Port GPIOA
#define FL_Pin GPIO_PIN_1
#define FL_GPIO_Port GPIOA
#define LED_SL_Pin GPIO_PIN_2
#define LED_SL_GPIO_Port GPIOA
#define LED_FL_Pin GPIO_PIN_3
#define LED_FL_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_5
#define BUZZ_GPIO_Port GPIOA
#define IN_L2_Pin GPIO_PIN_7
#define IN_L2_GPIO_Port GPIOA
#define IN_L1_Pin GPIO_PIN_4
#define IN_L1_GPIO_Port GPIOC
#define ADC_BATT_Pin GPIO_PIN_5
#define ADC_BATT_GPIO_Port GPIOC
#define FUN_Pin GPIO_PIN_11
#define FUN_GPIO_Port GPIOB
#define IN_R1_Pin GPIO_PIN_12
#define IN_R1_GPIO_Port GPIOB
#define IN_R2_Pin GPIO_PIN_13
#define IN_R2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_3
#define LED6_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOB
#define GYRO_SS_Pin GPIO_PIN_7
#define GYRO_SS_GPIO_Port GPIOB
#define LED_FR_Pin GPIO_PIN_8
#define LED_FR_GPIO_Port GPIOB
#define LED_SR_Pin GPIO_PIN_9
#define LED_SR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
