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
#define UserBtn_Pin GPIO_PIN_13
#define UserBtn_GPIO_Port GPIOC
#define UserBtn_EXTI_IRQn EXTI15_10_IRQn
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_14
#define LED5_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// Writes the same value (0/1) to the 3 built-in LEDs
#define WRITE_ALL_BOARD_LEDS(onOff) do {\
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, onOff);\
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, onOff);\
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, onOff);\
} while(0)

// Writes the lX value (0/1) to the lX external LED (5 in total)
#define WRITE_EXTERNAL_LEDS(l1, l2, l3, l4, l5) do {\
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, l1);\
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, l2);\
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, l3);\
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, l4);\
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, l5);\
} while (0)

// Writes the same value (0/1) to the 5 external LEDs
#define WRITE_ALL_EXTERNAL_LEDS(onOff) WRITE_EXTERNAL_LEDS(onOff, onOff, onOff, onOff, onOff)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
