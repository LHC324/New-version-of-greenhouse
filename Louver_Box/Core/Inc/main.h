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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*Terminal key value*/
#define ESC_CODE 0x1B
#define BACKSPACE_CODE 0x08
#define ENTER_CODE 0x0D
#define SPOT_CODE 0x2E
#define COMMA_CODE 0x2C
#define GET_TIMEOUT_FLAG(Stime, Ctime, timeout, MAX)                 \
  ((Ctime) < (Stime) ? !!((((MAX) - (Stime)) + (Ctime)) > (timeout)) \
                     : !!((Ctime) - (Stime) > (timeout)))
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
#define WAKEUP_Pin GPIO_PIN_1
#define WAKEUP_GPIO_Port GPIOA
#define RELOAD_Pin GPIO_PIN_1
#define RELOAD_GPIO_Port GPIOB
#define LORA_RELOAD_Pin GPIO_PIN_15
#define LORA_RELOAD_GPIO_Port GPIOB
#define LORA_WAKE_UP_Pin GPIO_PIN_8
#define LORA_WAKE_UP_GPIO_Port GPIOA
#define LORA_AUX_Pin GPIO_PIN_11
#define LORA_AUX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
