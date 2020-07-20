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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

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
#define Brake_Pin GPIO_PIN_0
#define Brake_GPIO_Port GPIOA
#define APPS_1_Pin GPIO_PIN_1
#define APPS_1_GPIO_Port GPIOA
#define APPS_2_Pin GPIO_PIN_2
#define APPS_2_GPIO_Port GPIOA
#define Brake_ON_OFF_Pin GPIO_PIN_5
#define Brake_ON_OFF_GPIO_Port GPIOA
#define Ready_to_Drive_Button_ON_OFF_Pin GPIO_PIN_7
#define Ready_to_Drive_Button_ON_OFF_GPIO_Port GPIOA
#define Ready_to_Drive_Button_ON_OFF_EXTI_IRQn EXTI9_5_IRQn
#define BrakeRangesOK_Pin GPIO_PIN_8
#define BrakeRangesOK_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_9
#define Buzzer_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_15
#define USART2_RX_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_3
#define USART2_TX_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
