/**
 * @brief	shell:STM32F743IIT6
 * @author	LHC
 * @date	2021/10/18
 */
#include "shell_port.h"
#include "usart.h"

/*定义shell目标端口*/
#if defined(USING_IO_UART)
#define SHELL_TARGET_UART S_Uart1
#elif defined(USING_USB)
#include "usbd_cdc_if.h"
#else
#define SHELL_TARGET_UART huart3
#endif

/*定义shell调试接口*/
#define Printf_Dbug(info) shellPrint(&shell, info);

#if defined(USING_RTTHREAD)
#include "rtthread.h"
extern rt_mutex_t shellMutexHandle;
#else
#include "cmsis_os.h"
extern osMutexId shellMutexHandle;
#endif
/* 定义shell对象*/
Shell shell;
char shell_buffer[SHELL_BUFFER_SIZE];

char shell_ch = '\0';
extern osSemaphoreId Shell_SemHandle;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* 判断是哪个串口触发的中断 */
	if (huart->Instance == USART3)
	{
		osSemaphoreRelease(Shell_SemHandle);
		// 使能串口中断接收
		HAL_UART_Receive_IT(&SHELL_TARGET_UART, (uint8_t *)&shell_ch, 1);
	}
}

/**
 * @brief shell读取数据函数原型
 *
 * @param data shell读取的字符
 * @param len 请求读取的字符数量
 *
 * @return unsigned short 实际读取到的字符数量
 */
unsigned short User_Shell_Read(char *data, unsigned short len)
{
	// #if defined(USING_IO_UART)
	// 	if (HAL_SUART_Receive(&SHELL_TARGET_UART, (uint8_t *)data, len, 0x100) != HAL_OK)
	// #elif defined(USING_USB)
	// 	if (!Usb.counts)
	// #else
	// 	if (HAL_UART_Receive(&SHELL_TARGET_UART, (uint8_t *)data, len, 0x01) != HAL_OK)
	// #endif
	// 	{ /*串口接收数据失败*/
	// 		len = 0;
	// 	}
	// #if defined(USING_USB)
	// 	else
	// 	{
	// 		// for (uint8_t *p = Usb.pRxbuf; p < Usb.pRxbuf + len;)
	// 		// 	*data++ = *p++;
	// 		// *data = Usb.pRxbuf[0];
	// 	}
	// #endif
	// 	return len;

	/*Direct reading is inefficient and special key values are easily lost*/
	if (osOK == osSemaphoreWait(Shell_SemHandle, osWaitForever))
	{
		*data = shell_ch;
		return len;
	}
	return 0;
}

#define FRAME_HEADER "\xFF\xFF\x00"
/**
 * @brief shell写数据函数原型
 *
 * @param data 需写的字符数据
 * @param len 需要写入的字符数
 *
 * @return unsigned short 实际写入的字符数量
 */
unsigned short User_Shell_Write(char *data, unsigned short len)
{
#if defined(USING_L101)
	char *pData = (char *)CUSTOM_MALLOC(sizeof(len + 3U));

	if (pData)
	{
		memcpy(pData, FRAME_HEADER, sizeof(FRAME_HEADER) - 1U);
		memcpy(&pData[3], data, len);
	}
	else
	{
		goto __exit;
	}
	if (HAL_UART_Transmit(&SHELL_TARGET_UART, (uint8_t *)pData, len + 3U, 0x100) != HAL_OK)
#else
#if defined(USING_IO_UART)
	if (HAL_SUART_Transmit(&SHELL_TARGET_UART, (uint8_t *)data, len, 0x100) != HAL_OK)
#elif defined(USING_USB)
	if (CDC_Transmit_FS((uint8_t *)data, len) != USBD_OK)
#else
	if (HAL_UART_Transmit(&SHELL_TARGET_UART, (uint8_t *)data, len, 0x100) != HAL_OK)
	// HAL_UART_Transmit_DMA(&SHELL_TARGET_UART, (uint8_t *)data, len);
	// while (__HAL_UART_GET_FLAG(&SHELL_TARGET_UART, UART_FLAG_TC) == RESET)
	// {
	// }
#endif
#endif
	{ /*串口发送数据失败*/
		len = 0;
	}
#if defined(USING_USB)
	extern bool g_oSRunFlag;
	if (g_oSRunFlag)
		osDelay(1);
	else
		HAL_Delay(1);
#endif

#if defined(USING_L101)
__exit:
	CUSTOM_FREE(pData);
#endif
	return len;
}

/**
 * @brief 用户shell上锁
 *
 * @param shell shell
 *
 * @return int 0
 */
int userShellLock(Shell *shell)
{
#if defined(USING_RTTHREAD)
	rt_mutex_take(shellMutexHandle, RT_WAITING_FOREVER);
#else
	osRecursiveMutexWait(shellMutexHandle, portMAX_DELAY);
#endif
	return 0;
}

/**
 * @brief 用户shell解锁
 *
 * @param shell shell
 *
 * @return int 0
 */
int userShellUnlock(Shell *shell)
{
#if defined(USING_RTTHREAD)
	/* 释 放 互 斥 锁 */
	rt_mutex_release(shellMutexHandle);
#else
	osRecursiveMutexRelease(shellMutexHandle);
#endif
	return 0;
}

/**
 * @brief shell初始化
 *
 * @param None
 *
 * @return None
 */
void MX_ShellInit(Shell *shell)
{
	shell->write = User_Shell_Write;
	shell->read = User_Shell_Read;
	shell->lock = userShellLock;
	shell->unlock = userShellUnlock;

	shellInit(shell, shell_buffer, SHELL_BUFFER_SIZE);
}
