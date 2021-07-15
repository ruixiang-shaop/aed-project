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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_PLACA1_ADDR 0x5B0
#define CAN_PLACA2_ADDR 0x5A0
#define btn2_Pin GPIO_PIN_2
#define btn2_GPIO_Port GPIOE
#define btn2_EXTI_IRQn EXTI2_IRQn
#define btn4_Pin GPIO_PIN_3
#define btn4_GPIO_Port GPIOE
#define btn4_EXTI_IRQn EXTI3_IRQn
#define btn3_Pin GPIO_PIN_5
#define btn3_GPIO_Port GPIOE
#define btn3_EXTI_IRQn EXTI9_5_IRQn
#define reset_Pin GPIO_PIN_13
#define reset_GPIO_Port GPIOC
#define reset_EXTI_IRQn EXTI15_10_IRQn
#define btn5_Pin GPIO_PIN_7
#define btn5_GPIO_Port GPIOF
#define btn5_EXTI_IRQn EXTI9_5_IRQn
#define led1_Pin GPIO_PIN_0
#define led1_GPIO_Port GPIOB
#define animation_Pin GPIO_PIN_1
#define animation_GPIO_Port GPIOG
#define animation_EXTI_IRQn EXTI1_IRQn
#define led3_Pin GPIO_PIN_14
#define led3_GPIO_Port GPIOB
#define btn1_Pin GPIO_PIN_4
#define btn1_GPIO_Port GPIOD
#define btn1_EXTI_IRQn EXTI4_IRQn
#define led2_Pin GPIO_PIN_7
#define led2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
