/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tool.h"
// #include "Modbus.h"
#include "small_modbus_port.h"
#include "lora.h"
#include "shell_port.h"
/*https://www.cnblogs.com/ydmblog/p/15308372.html*/
#include <cm_backtrace.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HARDWARE_VERSION "V1.0.0"
#define SOFTWARE_VERSION "V0.1.0"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId shellHandle;
osThreadId CpuHandle;
osThreadId LoraHandle;
osThreadId IWDGHandle;
osMutexId shellMutexHandle;
osMutexId modbusMutexHandle;
osSemaphoreId Cpu_ReciveHandle;
osSemaphoreId Lora_ReciveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Shell_Task(void const *argument);
void Cpu_Task(void const *argument);
void Lora_Task(void const *argument);
void IWDG_Task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
  shellPrint(Shell_Object, "@Error:%s is stack overflow!\r\n", pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created. It is also called by various parts of the
  demo application. If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  shellPrint(Shell_Object, "@Error:memory allocation failed!\r\n");
  shellPrint(Shell_Object, "\r\n\r\nOS remaining heap = %dByte, Mini heap = %dByte\r\n",
             xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  /*Solve the problem of scheduling stuck after running out of memory: reset MCU with Watchdog*/
  for (;;)
    ;
}

/**
 * @brief  Obtain card slot and board code
 * @param  None
 * @retval Card slot number and card type
 */
uint8_t get_card_id(void)
{
  GPIO_TypeDef *pGPIOx = CODE0_GPIO_Port;
  uint8_t data = 0x00; // 0xE0;
  /*PB8、PB9*/
  data |= (uint16_t)(((uint16_t)pGPIOx->IDR & 0x0300) >> 8U);
  /*PC13、PC14*/
  pGPIOx = CODE2_GPIO_Port;
  data |= (uint16_t)(((uint16_t)pGPIOx->IDR & 0x6000) >> 11U);
  /*PB0、PB1、PB2*/
  pGPIOx = TYPE0_GPIO_Port;
  data |= (uint16_t)(((uint16_t)pGPIOx->IDR & 0x0007) << 4U);
  /*PB12*/
  data |= (uint16_t)(((uint16_t)pGPIOx->IDR & 0x1000) >> 5U);
  return data;
}

/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of shellMutex */
  osMutexDef(shellMutex);
  shellMutexHandle = osMutexCreate(osMutex(shellMutex));

  /* definition and creation of modbusMutex */
  osMutexDef(modbusMutex);
  modbusMutexHandle = osMutexCreate(osMutex(modbusMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Cpu_Recive */
  osSemaphoreDef(Cpu_Recive);
  Cpu_ReciveHandle = osSemaphoreCreate(osSemaphore(Cpu_Recive), 1);

  /* definition and creation of Lora_Recive */
  osSemaphoreDef(Lora_Recive);
  Lora_ReciveHandle = osSemaphoreCreate(osSemaphore(Lora_Recive), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  MX_ShellInit(Shell_Object);
  MX_ModbusInit();
  MX_Lora_Init();
#if defined(USING_CMBACKTRACE)
  SCB->CCR |= (1 << 3); /* bit3: UNALIGN_TRP. */
  SCB->CCR |= (1 << 4); /* bit4: DIV_0_TRP. */
  /* CmBacktrace initialize */
  cm_backtrace_init("communication1", HARDWARE_VERSION, SOFTWARE_VERSION);
#endif
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of shell */
  osThreadDef(shell, Shell_Task, osPriorityLow, 0, 192);
  shellHandle = osThreadCreate(osThread(shell), (void *)Shell_Object);

  /* definition and creation of Cpu */
  osThreadDef(Cpu, Cpu_Task, osPriorityNormal, 0, 256);
  CpuHandle = osThreadCreate(osThread(Cpu), (void *)Modbus_Object);

  /* definition and creation of Lora */
  osThreadDef(Lora, Lora_Task, osPriorityHigh, 0, 256);
  LoraHandle = osThreadCreate(osThread(Lora), (void *)Lora_Object);

  /* definition and creation of IWDG */
  osThreadDef(IWDG, IWDG_Task, osPriorityRealtime, 0, 64);
  IWDGHandle = osThreadCreate(osThread(IWDG), (void *)Shell_Object);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_Shell_Task */
/**
 * @brief  Function implementing the shell thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Shell_Task */
void Shell_Task(void const *argument)
{
  /* USER CODE BEGIN Shell_Task */
  // Shell *shell = (Shell *)argument;
  // char data = '\0';
  /* Infinite loop */
  for (;;)
  {
    shellTask((void *)argument);
  }
  /* USER CODE END Shell_Task */
}

/* USER CODE BEGIN Header_Cpu_Task */
/**
 * @brief	rt_thread 初始化目标串口的串口空闲中断+DMA接收
 * @details 乒乓缓冲实现方式：https://www.cnblogs.com/puyu9495/p/15914090.html
 * @param	puart Uartx handle
 * @param p_semaphore signal handler
 * @retval  none
 */
static void free_rtos_hal_uartx_dma_info_init(void *p_semaphore, pUartHandle puart)
{
  /*Initialize target serial port DMA configuration*/
  if (puart && puart->huart && puart->phdma && puart->rx.pbuf)
  {
    __HAL_UART_ENABLE_IT((UART_HandleTypeDef *)puart->huart, UART_IT_IDLE);
    /*DMA buffer must point to an entity address!!!*/
    HAL_UART_Receive_DMA((UART_HandleTypeDef *)puart->huart, puart->rx.pbuf, puart->rx.size);
    puart->rx.count = 0;
    memset(puart->rx.pbuf, 0x00, puart->rx.size);
  }
  if (p_semaphore)
    puart->semaphore = p_semaphore;
}
/**
 * @brief Function implementing the Cpu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Cpu_Task */
void Cpu_Task(void const *argument)
{
  /* USER CODE BEGIN Cpu_Task */
  pModbusHandle pd = (pModbusHandle)argument;
  if (pd)
  {
    free_rtos_hal_uartx_dma_info_init(Cpu_ReciveHandle, &pd->Uart);
    osSemaphoreWait(pd->Uart.semaphore, osWaitForever);
  }

  /* Infinite loop */
  for (;;)
  {
    /*https://www.cnblogs.com/w-smile/p/11333950.html*/
    if ((osOK == osSemaphoreWait(pd->Uart.semaphore, osWaitForever)) && pd)
    {
      small_modbus_handler(pd);
#if defined(USING_DEBUG)
      shellPrint(Shell_Object, "modbus received a packet of data.\r\n");
#endif
    }
//	  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
//	  HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, "hello world.\r\n",strlen("hello world.\r\n"), 0x100);
//	  osDelay(500);
  }
  /* USER CODE END Cpu_Task */
}

/* USER CODE BEGIN Header_Lora_Task */
/**
 * @brief Function implementing the Lora thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Lora_Task */
void Lora_Task(void const *argument)
{
  /* USER CODE BEGIN Lora_Task */
  pLoraHandle pl = (pLoraHandle)argument;
  if (pl)
  {
    free_rtos_hal_uartx_dma_info_init(Lora_ReciveHandle, &pl->Uart);
    osSemaphoreWait(pl->Uart.semaphore, osWaitForever);
  }

  /* Infinite loop */
  for (;;)
  {
    /*https://www.cnblogs.com/w-smile/p/11333950.html*/
    if ((osOK == osSemaphoreWait(pl->Uart.semaphore, LORA_SCHEDULE_TIMES)) && pl)
    {
      if (pl->Mode == Lora_Nomal_Recive)
        pl->Lora_Recive_Poll(pl);
#if defined(USING_DEBUG)
      shellPrint(Shell_Object, "Lora received a packet of data.\r\n");
#endif
    }
    else
    {
      if (pl->Mode == Lora_Nomal_Recive)
        pl->Lora_Transmit_Poll(pl);
    }
// osDelay(1000);
#if defined(USING_DEBUG)
#if ((configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS > 0) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))
    TaskStatus_t *pTable = CUSTOM_MALLOC(uxTaskGetNumberOfTasks() * sizeof(TaskStatus_t));
    if (pTable)
    {
      vTaskList((char *)pTable);
      shellPrint(Shell_Object, "TaskName\tTaskState Priority   RemainingStack TaskID\r\n");
      shellPrint(Shell_Object, "%s\r\n", pTable);
    }
    CUSTOM_FREE(pTable);
    shellPrint(Shell_Object, "\r\nremaining heap = %d.\r\n", xPortGetFreeHeapSize());
#endif
#endif
    // shellPrint(Shell_Object, "\r\n\r\nOS remaining heap = %dByte, Mini heap = %dByte\r\n",
    //            xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  }
  /* USER CODE END Lora_Task */
}

/* USER CODE BEGIN Header_IWDG_Task */
typedef enum
{
  irq_start,
  irq_continue,
  irq_end,
} irq_state;

typedef struct
{
  irq_state status : 7;
  GPIO_PinState irq : 1;
  uint8_t first_flag : 1;
  uint16_t counter;
} irq_sm_handle;

irq_sm_handle irq_sm;
/*First power-on sign*/
bool first_boot = false;

uint16_t TIRQ_Handle(void)
{
#define IRQ_BASE_TIMES (200U)
#define IRQ_OFFSET_TIMES (20U)
#define IRQ_LOW_TIMES (10U)
  /*Interrupt signal generation*/
  pModbusHandle pd = Modbus_Object;
  irq_sm_handle *const psm = &irq_sm;

  if (NULL == pd->Slave.pHandle)
    return IRQ_BASE_TIMES;

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
    psm->first_flag = true;
    if (!first_boot)
    {
      first_boot = true;
      psm->irq = GPIO_PIN_SET;
      return (pd->Slave.id
                  ? IRQ_BASE_TIMES + (pd->Slave.id * IRQ_OFFSET_TIMES)
                  : IRQ_BASE_TIMES);
    }
    else
      psm->irq = GPIO_PIN_RESET;
    psm->counter = IRQ_LOW_TIMES;
    psm->status = irq_continue;
  }
  break;
  case irq_continue:
  {
    psm->irq = GPIO_PIN_SET;
    psm->counter = IRQ_BASE_TIMES;
    psm->status = irq_start;
  }
  break;
  case irq_end:
  {
    psm->first_flag = false;
    psm->irq = GPIO_PIN_SET;
  }
  break;
  }

  HAL_GPIO_WritePin(M_IRQ_GPIO_Port, M_IRQ_Pin, psm->irq);
  return psm->counter;
}
/**
 * @brief Function implementing the IWDG thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IWDG_Task */
void IWDG_Task(void const *argument)
{
  /* USER CODE BEGIN IWDG_Task */
  /*Wait for the main cpu to start:Basic delay with one more cycle*/
  osDelay(1000); //(1000 - IRQ_BASE_TIMES)
  /* Infinite loop */
  for (;;)
  {
    extern IWDG_HandleTypeDef hiwdg;
    HAL_IWDG_Refresh(&hiwdg);
    /*External interruption*/
    osDelay(TIRQ_Handle());
  }
  /* USER CODE END IWDG_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
