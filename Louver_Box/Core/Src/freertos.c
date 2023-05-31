/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*https://www.cnblogs.com/ydmblog/p/15308372.html*/
#include <cm_backtrace.h>
#include "shell_port.h"
#include "small_modbus_port.h"
#include "lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
osThreadId loraHandle;
osThreadId data_collectionHandle;
osThreadId iwdgHandle;
osTimerId Timer1Handle;
osMutexId shellMutexHandle;
osMutexId modbusMutexHandle;
osSemaphoreId Comm_ReciveHandle;
osSemaphoreId Rs485_ReciveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Shell_Task(void const *argument);
void Lora_Handle_Task(void const *argument);
void Data_Collection_Task(void const *argument);
void IWDG_Task(void const *argument);
void Timer_Callback(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
  shellPrint(&shell, "@error: %s is stack overflow!\r\n", pcTaskName);
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
  shellPrint(&shell, "@error: memory allocation failed!\r\nremaining\tmini\r\n\
  %#x, %#X\r\n",
             xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  MX_ShellInit(&shell);
  MX_ModbusInit();
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
  /* definition and creation of Comm_Recive */
  osSemaphoreDef(Comm_Recive);
  Comm_ReciveHandle = osSemaphoreCreate(osSemaphore(Comm_Recive), 1);

  /* definition and creation of Rs485_Recive */
  osSemaphoreDef(Rs485_Recive);
  Rs485_ReciveHandle = osSemaphoreCreate(osSemaphore(Rs485_Recive), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Timer1 */
  osTimerDef(Timer1, Timer_Callback);
  Timer1Handle = osTimerCreate(osTimer(Timer1), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of shell */
  osThreadDef(shell, Shell_Task, osPriorityLow, 0, 256);
  shellHandle = osThreadCreate(osThread(shell), (void *)&shell);

  /* definition and creation of lora */
  osThreadDef(lora, Lora_Handle_Task, osPriorityHigh, 0, 128);
  loraHandle = osThreadCreate(osThread(lora), (void *)Modbus_Object);

  /* definition and creation of data_collection */
  osThreadDef(data_collection, Data_Collection_Task, osPriorityAboveNormal, 0, 128);
  data_collectionHandle = osThreadCreate(osThread(data_collection), (void *)Modbus_Object);

  /* definition and creation of iwdg */
  osThreadDef(iwdg, IWDG_Task, osPriorityRealtime, 0, 64);
  iwdgHandle = osThreadCreate(osThread(iwdg), NULL);

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
  Shell *pshell = (Shell *)argument;

  /* Infinite loop */
  for (;;)
  {
    shellTask((void *)argument);
  }
  /* USER CODE END Shell_Task */
}

/* USER CODE BEGIN Header_Lora_Handle_Task */
extern bool g_LoraMutex;

/**
 * @brief	获取Lora模块当前是否空闲
 * @details	无线发�?�数据时拉低，用于指示发送繁忙状�?
 * @param	None
 * @retval	ture/fale
 */
static bool __inline Get_Lora_Status(void)
{
  return ((bool)HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin));
}

// void Lora_Data_Handle(pUartHandle pu, pModbusHandle pd)
// {
// #define LOUVER_BOX_ID 13U

//   uint16_t crc16 = 0;

//   if (NULL == pu || NULL == pd)
//     return;

//   if (pu->rx.count <= 5U)
//     return;

//   if (pu->rx.pbuf[3] != LOUVER_BOX_ID)
//     return;

//   crc16 = get_crc16(pu->rx.pbuf, pu->rx.count - 2U, 0xffff);

//   if (Get_Smodbus_Data(pu->rx.pbuf, pu->rx.count - 2U, SMODBUS_WORD) !=
//       ((uint16_t)((crc16 >> 8U) | (crc16 << 8U))))
//     return;

//   uint8_t len = pu->rx.pbuf[5U + 3U] * 2U;
//   uint8_t buf[24U] = {0, 0, 0, LOUVER_BOX_ID, ReadInputReg, len};

//   if (len < sizeof(buf) / sizeof(buf[0]) &&
//       !pd->Mod_Operatex(pd, InputRegister, Read, 0x0,
//                         &buf[6U], len))
//   {
// #if LORA_USING_DEBUG
//     LORA_DEBUG("Input coil write failed!\r\n");
// #endif
//   }
//   memmove(buf, pu->rx.pbuf, 3U);
// #if defined(USING_REPEATER_MODE)
//   buf[1] = buf[2] = 0x01;
//   buf[3] |= 0x80;
// #else

// #endif
//   pu->tx.count = 3U + 3U + 2U + len;
//   crc16 = get_crc16(buf, pu->tx.count - 2U, 0xffff);

//   memmove(&buf[sizeof(buf) - sizeof(uint16_t)], &crc16, sizeof(uint16_t));

//   /*Ensure that the L101 module is currently idle*/
//   // while (!Get_Lora_Status())
//   // {
//   //   osDelay(1);
//   // }
//   HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pu->huart, buf, pu->tx.count);
//   while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pu->huart, UART_FLAG_TC) == RESET)
//   {
//   }
// #undef LOUVER_BOX_ID
// }

// void Lora_Data_Handle(pUartHandle pu, pModbusHandle pd)
// {
// #define LOUVER_BOX_ID 13U

//   uint16_t crc16 = 0;

//   if (NULL == pu || NULL == pd)
//     return;

//   if (pu->rx.count <= 5U)
//     return;

//   if (pu->rx.pbuf[0] != LOUVER_BOX_ID)
//     return;

//   crc16 = get_crc16(pu->rx.pbuf, pu->rx.count - 2U, 0xffff);

//   if (Get_Smodbus_Data(pu->rx.pbuf, pu->rx.count - 2U, SMODBUS_WORD) !=
//       ((uint16_t)((crc16 >> 8U) | (crc16 << 8U))))
//     return;

//   uint8_t len = pu->rx.pbuf[5U] * 2U;
//   uint8_t buf[24U] = {0, 0, 0, LOUVER_BOX_ID, ReadInputReg, len};

//   if (len < sizeof(buf) / sizeof(buf[0]) &&
//       !pd->Mod_Operatex(pd, InputRegister, Read, 0x0,
//                         &buf[6U], len))
//   {
// #if LORA_USING_DEBUG
//     LORA_DEBUG("Input coil write failed!\r\n");
// #endif
//   }
//   // memmove(buf, pu->rx.pbuf, 3U);
// #if defined(USING_REPEATER_MODE)
//   buf[1] = buf[2] = 0x01;
//   // buf[3] |= 0x80;
// #else

// #endif
//   pu->tx.count = 3U + 3U + 2U + len;
//   /*The first three bytes do not participate in the crc calculation*/
//   crc16 = get_crc16(&buf[3], pu->tx.count - 5U, 0xffff);

//   memmove(&buf[sizeof(buf) - sizeof(uint16_t)], &crc16, sizeof(uint16_t));

//   /*Ensure that the L101 module is currently idle*/
//   // while (!Get_Lora_Status())
//   // {
//   //   osDelay(1);
//   // }
//   HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pu->huart, buf, pu->tx.count);
//   while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pu->huart, UART_FLAG_TC) == RESET)
//   {
//   }
// #undef LOUVER_BOX_ID
// }

void Lora_Data_Handle(pUartHandle pu, pModbusHandle pd)
{
#define LOUVER_BOX_ID 2U

  uint16_t crc16 = 0;

  if (NULL == pu || NULL == pd)
    return;

  if (pu->rx.count <= 5U)
    return;

  if (pu->rx.pbuf[0] != LOUVER_BOX_ID)
    return;

  crc16 = get_crc16(pu->rx.pbuf, pu->rx.count - 2U, 0xffff);

  if (Get_Smodbus_Data(pu->rx.pbuf, pu->rx.count - 2U, SMODBUS_WORD) !=
      ((uint16_t)((crc16 >> 8U) | (crc16 << 8U))))
    return;

  uint8_t len = pu->rx.pbuf[5U] * 2U;
  uint8_t buf[21U] = {LOUVER_BOX_ID, ReadInputReg, len};

  if (len < sizeof(buf) / sizeof(buf[0]) &&
      !pd->Mod_Operatex(pd, InputRegister, Read, 0x0,
                        &buf[3U], len))
  {
#if LORA_USING_DEBUG
    LORA_DEBUG("Input coil write failed!\r\n");
#endif
  }

// #if defined(USING_REPEATER_MODE)
//   buf[1] = buf[2] = 0x01;
//   // buf[3] |= 0x80;
// #else

// #endif
  pu->tx.count =3U + 2U + len;
  /*The first three bytes do not participate in the crc calculation*/
  crc16 = get_crc16(buf, pu->tx.count - 2U, 0xffff);

  memmove(&buf[sizeof(buf) - sizeof(uint16_t)], &crc16, sizeof(uint16_t));

  HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pu->huart, buf, pu->tx.count);
  while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pu->huart, UART_FLAG_TC) == RESET)
  {
  }
#undef LOUVER_BOX_ID
}

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

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

// uint8_t Louver_Tx[32U];
uint8_t Louver_Rx[32U];
UartHandle Louver_Uart = {
    .huart = &huart1,
    .phdma = &hdma_usart1_rx,
    .tx = {
        // .pbuf = sizeof(Louver_Tx) / sizeof(Louver_Tx[0]),
        .pbuf = NULL,
        .count = 0,
    },
    .rx = {
        .pbuf = Louver_Rx,
        .size = sizeof(Louver_Rx) / sizeof(Louver_Rx[0]),
    },
};

/**
 * @brief Function implementing the lora thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Lora_Handle_Task */
void Lora_Handle_Task(void const *argument)
{
  /* USER CODE BEGIN Lora_Handle_Task */
  pModbusHandle pd = (pModbusHandle)argument;
  UartHandle *pu = &Louver_Uart;

  while (NULL == pd)
    ;

  free_rtos_hal_uartx_dma_info_init(Comm_ReciveHandle, pu);
  /*The number of resources available after the creation of binary semaphore is 1,
which needs to be released first*/
  osSemaphoreWait(pu->semaphore, osWaitForever);

  /* Infinite loop */
  for (;;)
  {
    /*https://www.cnblogs.com/w-smile/p/11333950.html*/
    if (osOK == osSemaphoreWait(pu->semaphore, osWaitForever))
    {
      if (!g_LoraMutex)
      {
        Lora_Data_Handle(pu, pd);
      }
    }
  }
  /* USER CODE END Lora_Handle_Task */
}

/* USER CODE BEGIN Header_Data_Collection_Task */

/**
 * @brief Request shutter box data.
 * @param argument: Not used
 * @retval None
 */
// void Request_Louver_Box_Data(UartHandle *pu)
// {
//   if (NULL == pu)
//     return;

//   uint8_t buf[] = {SMALL_MODBUS_SLAVE_ADDR, 0x03, 0x01, 0xF4, 0x00, 0x08, 0x00, 0x00};
//   uint16_t crc = get_crc16(buf, sizeof(buf) - 2U, 0xFFFF);

//   memcpy(&buf[sizeof(buf) - 2U], (uint8_t *)&crc, sizeof(uint16_t));

//   HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pu->huart, buf, sizeof(buf));
//   while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pu->huart, UART_FLAG_TC) == RESET)
//   {
//   }
// }

/**
 * @brief Function implementing the data_collection thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Data_Collection_Task */
void Data_Collection_Task(void const *argument)
{
  /* USER CODE BEGIN Data_Collection_Task */

#define Louver_Box_Regs_Addr 500U
  pModbusHandle pd = (pModbusHandle)argument;
  while (NULL == pd)
    ;

  Request_HandleTypeDef modbus_master_cfg = {
      .code = ReadInputReg,
      .reg_start_addr = Louver_Box_Regs_Addr,
      .reg_len = 0x08,
  };
  memmove(&pd->Master.request_data, &modbus_master_cfg, sizeof(modbus_master_cfg));

  free_rtos_hal_uartx_dma_info_init(Rs485_ReciveHandle, &pd->Uart);
  /*The number of resources available after the creation of binary semaphore is 1,
  which needs to be released first*/
  osSemaphoreWait(pd->Uart.semaphore, osWaitForever);

  /* Infinite loop */
  for (;;)
  {
    /*https://www.cnblogs.com/w-smile/p/11333950.html*/
    if ((osOK == osSemaphoreWait(pd->Uart.semaphore, 1000U)))
    {
      if (!g_LoraMutex)
      {
        // pd->Slave.id = LOUVER_BOX_ID;
        // pd->Uart.rx.count = pu->rx.count;
        // memcpy(pu->rx.pbuf, pd->Uart.rx.pbuf, pd->Uart.rx.count);
        // pu->rx.count = 0;

        /*Offset Address*/
        pd->Master.request_data.reg_start_addr = 0;
        small_modbus_handler(pd);
      }
    }
    else
    {
      if (!g_LoraMutex)
      {
        // Request_Louver_Box_Data(pu);
        pd->Master.request_data.reg_start_addr = Louver_Box_Regs_Addr;
        pd->Mod_Request(pd);
      }
    }
    // osDelay(1000);
  }
  /* USER CODE END Data_Collection_Task */
}

/* USER CODE BEGIN Header_IWDG_Task */
/**
 * @brief Function implementing the IWDG thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IWDG_Task */
void IWDG_Task(void const *argument)
{
  /* USER CODE BEGIN IWDG_Task */
  /* Infinite loop */
  for (;;)
  {
    extern IWDG_HandleTypeDef hiwdg;

    // HAL_IWDG_Refresh(&hiwdg);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(1000);
  }
  /* USER CODE END IWDG_Task */
}

/* Timer_Callback function */
void Timer_Callback(void const *argument)
{
  /* USER CODE BEGIN Timer_Callback */

  /* USER CODE END Timer_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
