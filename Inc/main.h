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
#include "stm32l0xx_hal.h"

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
#define ADC_CONVERTIONCOUNT 5
#define ID_Key_Pin GPIO_PIN_0
#define ID_Key_GPIO_Port GPIOA
#define ID_Key_EXTI_IRQn EXTI0_1_IRQn
#define O_LED_PWM_Pin GPIO_PIN_1
#define O_LED_PWM_GPIO_Port GPIOA
#define ID_Charge_En_Pin GPIO_PIN_4
#define ID_Charge_En_GPIO_Port GPIOA
#define IA_VBat_Pin GPIO_PIN_5
#define IA_VBat_GPIO_Port GPIOA
#define IA_LED_Current_Pin GPIO_PIN_6
#define IA_LED_Current_GPIO_Port GPIOA
#define IA_LED_Temperature_Pin GPIO_PIN_7
#define IA_LED_Temperature_GPIO_Port GPIOA
#define O_StatusLED_Pin GPIO_PIN_1
#define O_StatusLED_GPIO_Port GPIOB
#define O_VBat_En_Pin GPIO_PIN_6
#define O_VBat_En_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
