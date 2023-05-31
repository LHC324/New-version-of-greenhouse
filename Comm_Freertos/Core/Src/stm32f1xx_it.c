/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "small_modbus_port.h"
#include "shell_port.h"
#include "lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */
#if !defined(USING_CMBACKTRACE)
/*Hard Fault handler in C, with stack frame location and LR value
extracted from the assembly wrapper as input parameters*/
void hard_fault_handler_c(unsigned int *hardfault_args, unsigned lr_value)
{
  Shell *pshell = shellGetCurrent();
  unsigned int r0 = ((unsigned long)hardfault_args[0]);
  unsigned int r1 = ((unsigned long)hardfault_args[1]);
  unsigned int r2 = ((unsigned long)hardfault_args[2]);
  unsigned int r3 = ((unsigned long)hardfault_args[3]);
  unsigned int r12 = ((unsigned long)hardfault_args[4]);
  unsigned int lr = ((unsigned long)hardfault_args[5]);
  unsigned int pc = ((unsigned long)hardfault_args[6]);
  unsigned int psr = ((unsigned long)hardfault_args[7]);
  if (pshell)
    shellPrint(pshell, "[Hard fault handler]\nR0\t\tR1\t\tR2\t\tR3\
    \t\tR12\t\tLR\t\tPC\t\tPSR\t\tLR\r\n%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\t%x\r\n",
               r0, r1, r2, r3, r12, lr, pc, psr, lr_value);
}
#endif

// __asm void HardFault_Handler(void)
// {
// /* USER CODE BEGIN HardFault_IRQn 0 */
// #if !defined(USING_CMBACKTRACE)
//     MOVS r0, #4
//     MOV r1, LR
//     TST r0, r1
//     BEQ stacking_used_MSP
//     MRS R0, PSP ;first parameter - stacking was using PSP
//     B get_LR_and_branch
// stacking_used_MSP
//     MRS R0,MSP ;first parameter - stacking was using MSP
// get_LR_and_branch
//     MOV R1,LR ;second parameter is LR current value
//     LDR R2,=__cpp(hard_fault_handler_c)
//     BX R2
// #else
//     MOV     r0, lr                  ; get lr
//     MOV     r1, sp                  ; get stack pointer (current is MSP)
//     BL      cm_backtrace_fault

// Fault_Loop
//     BL      Fault_Loop              ;while(1)
// #endif
//   /* USER CODE END HardFault_IRQn 0 */
//   // while (1)
//   // {
//   //   /* USER CODE BEGIN W1_HardFault_IRQn 0 */
//   //   /* USER CODE END W1_HardFault_IRQn 0 */
//   // }
// }

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__weak void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
// #if !defined(USING_CMBACKTRACE)
//     MOVS r0, #4
//     MOV r1, LR
//     TST r0, r1
//     BEQ stacking_used_MSP
//     MRS R0, PSP ;first parameter - stacking was using PSP
//     B get_LR_and_branch
// stacking_used_MSP
//     MRS R0,MSP ;first parameter - stacking was using MSP
// get_LR_and_branch
//     MOV R1,LR ;second parameter is LR current value
//     LDR R2,=__cpp(hard_fault_handler_c)
//     BX R2
// #else
//     MOV     r0, lr                  ; get lr
//     MOV     r1, sp                  ; get stack pointer (current is MSP)
//     BL      cm_backtrace_fault

// Fault_Loop
//     BL      Fault_Loop              ;while(1)
// #endif
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  pModbusHandle pd = Modbus_Object;

  if (pd)
    uartx_recive_handle(&pd->Uart);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  pLoraHandle pl = Lora_Object;

  if (pl)
    uartx_recive_handle(&pl->Uart);
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
