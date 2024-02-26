/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define Motor1_dir_Pin GPIO_PIN_1
#define Motor1_dir_GPIO_Port GPIOA
#define Motor2_dir_Pin GPIO_PIN_2
#define Motor2_dir_GPIO_Port GPIOA
#define Button1_Plus_Pin GPIO_PIN_3
#define Button1_Plus_GPIO_Port GPIOA
#define Button1_Plus_EXTI_IRQn EXTI3_IRQn
#define Button1_Minus_Pin GPIO_PIN_4
#define Button1_Minus_GPIO_Port GPIOA
#define Button1_Minus_EXTI_IRQn EXTI4_IRQn
#define Button1_Rev_Pin GPIO_PIN_5
#define Button1_Rev_GPIO_Port GPIOA
#define Button1_Rev_EXTI_IRQn EXTI9_5_IRQn
#define Button2_Plus_Pin GPIO_PIN_6
#define Button2_Plus_GPIO_Port GPIOA
#define Button2_Plus_EXTI_IRQn EXTI9_5_IRQn
#define Button2_Minus_Pin GPIO_PIN_7
#define Button2_Minus_GPIO_Port GPIOA
#define Button2_Minus_EXTI_IRQn EXTI9_5_IRQn
#define Button2_Rev_Pin GPIO_PIN_0
#define Button2_Rev_GPIO_Port GPIOB
#define Button2_Rev_EXTI_IRQn EXTI0_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define PWM_Motor_1_Pin GPIO_PIN_8
#define PWM_Motor_1_GPIO_Port GPIOA
#define PWM_Motor_2_Pin GPIO_PIN_9
#define PWM_Motor_2_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
