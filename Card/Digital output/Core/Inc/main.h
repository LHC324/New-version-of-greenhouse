/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//#define USING_DEBUG
#define CUSTOM_MALLOC malloc
#define CUSTOM_FREE free
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
#define Q0_Pin GPIO_PIN_0
#define Q0_GPIO_Port GPIOA
#define Q1_Pin GPIO_PIN_1
#define Q1_GPIO_Port GPIOA
#define Q2_Pin GPIO_PIN_2
#define Q2_GPIO_Port GPIOA
#define Q3_Pin GPIO_PIN_3
#define Q3_GPIO_Port GPIOA
#define Q4_Pin GPIO_PIN_4
#define Q4_GPIO_Port GPIOA
#define Q5_Pin GPIO_PIN_5
#define Q5_GPIO_Port GPIOA
#define Q6_Pin GPIO_PIN_6
#define Q6_GPIO_Port GPIOA
#define Q7_Pin GPIO_PIN_7
#define Q7_GPIO_Port GPIOA
#define SO_Pin GPIO_PIN_1
#define SO_GPIO_Port GPIOB
#define CPU_TX_Pin GPIO_PIN_9
#define CPU_TX_GPIO_Port GPIOA
#define CPU_RX_Pin GPIO_PIN_10
#define CPU_RX_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_13
#define A_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_14
#define B_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
