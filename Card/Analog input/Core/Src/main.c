/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tool.h"
#include "Modbus.h"
#include "io_signal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  irq_start,
  irq_continue,
  irq_end,
} irq_state;

typedef struct
{
  irq_state status : 7;
  IRQ_Type irq : 1;
  uint8_t first_flag : 1;
  uint16_t counter;
} irq_sm_handle;

irq_sm_handle irq_sm;

void TIRQ_Handle(void *arg)
{
#define IRQ_BASE_TIMES (200U)
#define IRQ_OFFSET_TIMES (2U)
#define IRQ_LOW_TIMES (1U)
  /*Interrupt signal generation*/
  pModbusHandle pd = Modbus_Object;
  SoftTimer *pt = (SoftTimer *)arg;
  irq_sm_handle *const psm = &irq_sm;

  if (NULL == pd->Slave.pHandle ||
      NULL == pt)
    return;

  /*The board model is read or the corresponding interrupt processing request is responded*/
  psm->status = *(bool *)pd->Slave.pHandle
                    ? irq_end
                : psm->first_flag
                    ? psm->status
                    : irq_start;

  switch (psm->status)
  {
  case irq_start:
  {
    psm->irq = Enable;
    psm->counter = IRQ_LOW_TIMES;
    psm->first_flag = true;
    psm->status = irq_continue;
  }
  break;
  case irq_continue:
  {
    psm->irq = DisEnable;
    psm->counter = IRQ_BASE_TIMES;
    psm->status = irq_start;
  }
  break;
  case irq_end:
  {
    psm->first_flag = false;
    psm->irq = DisEnable;
  }
  break;
  }
  pt->target_counter = psm->counter;
  IRQ_Handle(psm->irq);
}

static void Debug_Handle(void *arg)
{
#if defined(USING_DEBUG)

#endif
}

SoftTimer timer_map[] = {
    {.run_flag = true, .enable = true, .execute_flag = false, .event = TIRQ_Handle, .target_counter = 100U},
#if defined(USING_DEBUG)
    {.run_flag = false, .enable = true, .execute_flag = false, .event = Debug_Handle, .target_counter = 100U},
    {.run_flag = false, .enable = true, .execute_flag = false, .event = Read_Analog_Io, .target_counter = 100U}
#else
    {.run_flag = false, .enable = true, .execute_flag = false, .event = Modbus_Handle, .target_counter = 1U},
    {.run_flag = false, .enable = true, .execute_flag = false, .event = Read_Analog_Io, .target_counter = 200U},
#endif
    {.counter = 0U, .enable = false, .execute_flag = false, .event = Debug_Handle, .target_counter = 1000U},
};
Timer_HandleTypeDef stimer = {
    .pmap = timer_map,
    .numbers = sizeof(timer_map) / sizeof(SoftTimer),
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#if defined(USING_DEBUG)
  HAL_Delay(5000);
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)Adc_buffer, ADC_DMA_SIZE);
  ptimer = &stimer;
  MX_ModbusInit();
  if (Modbus_Object)
  {
    __HAL_UART_ENABLE_IT(Modbus_Object->huart, UART_IT_IDLE);
    /*DMA buffer must point to an entity address!!!*/
    HAL_UART_Receive_DMA(Modbus_Object->huart, Modbus_Object->Slave.pRbuf, Modbus_Object->Slave.RxSize);
    Modbus_Object->Slave.RxCount = 0U;
    ptimer->pmap[0].arg = &ptimer->pmap[0];
    ptimer->pmap[0].target_counter = Modbus_Object->Slave_Id
                                         ? IRQ_BASE_TIMES + (Modbus_Object->Slave_Id * IRQ_OFFSET_TIMES)
                                         : IRQ_BASE_TIMES;
  }
  /* Enable receive error interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
  /*Wait for the main cpu to start*/
  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Timer_Handle();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*https://blog.csdn.net/mickey35/article/details/78529637*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  MX_USART1_UART_Init();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
