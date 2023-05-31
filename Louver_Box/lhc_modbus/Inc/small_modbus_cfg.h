/*
 * Copyright (c) 2006-2022, LHC Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_
#define PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*****************************************************功能配置区*******************************************/
/*Configuration Wizard：https://blog.csdn.net/qq_15647227/article/details/89297207*/
// <<< Use Configuration Wizard in Context Menu >>>

// <o>Selsect lhc modbus thread Running Environment
//  <i>Default: 2
//  <0=> Bare pager
//  <1=> Freertos
//  <2=> rt_thread
/*small_modbus使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define SMODBUS_USING_RTOS 1

// <s>Valid when using rtthread
//  <i>Device name
#define LHC_MODBUS_DEVICE_NAME "uart1"

// <o>Set lhc modbus Dynamic Memory allocation mode
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*small_modbus动态内存分配[0:不使用；1：使用]*/
#define SMODBUS_USING_MALLOC 1

// <o>Set lhc modbus Dma peripheral
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*small_modbusDMA选项*/
#define SMODBUS_USING_DMA 1

// <o>Set lhc modbus crc algorithm
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*small_modbusCRC校验*/
#define SMODBUS_USING_CRC 1

// <o>Selsect lhc modbus Debug Options
//  <i>Default: 2
//  <0=> Unused debug
//  <1=> leeter shell
//  <2=> finish shell
/*small_modbus调试选项[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
#define SMODBUS_USING_DEBUG 1

/*small_modbus调试输出终端选择:[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
// #define SMODBUS_USING_SHELL 2

// <o>Set the buffer size of lhc modbus receiving thread
//  <i>Default: 128 (Unit: byte)
//  <0-1024>
/*small_modbus数据缓冲区尺寸*/
#define SMODBUS_RX_BUF_SIZE 128

// <o>Set the buffer size of lhc modbus sending thread
//  <i>Default: 128 (Unit: byte)
//  <0-1024>
#define SMODBUS_TX_BUF_SIZE 128

#if (SMODBUS_USING_MALLOC)
#if (SMODBUS_USING_RTOS == 1)
#define smd_malloc pvPortMalloc
#define smd_free vPortFree
#elif (SMODBUS_USING_RTOS == 2)
#define smd_malloc rt_malloc
#define smd_free rt_free
#endif
#endif

// <o>Set lhc modbus master address
//  <i>Default: 1
//  <0-255>
/*协议栈参数配置*/
#define SMALL_MODBUS_MASTER_ADDR 0x01

    // <o>Set lhc modbus slave address
    //  <i>Default: 2
    //  <0-255>

#define SMALL_MODBUS_SLAVE_ADDR 0x02
// <o>Set the number of registers in the lhc modbus register pool
//  <i>Default: 128
//  <0-1024>
/*寄存器池尺寸*/
#define SMODBUS_REG_POOL_SIZE 32U

#define COIL_OFFSET (1)
#define INPUT_COIL_OFFSET (10001)
#define INPUT_REGISTER_OFFSET (30001)
#define HOLD_REGISTER_OFFSET (40001)

// <c1>Enable lhc modbus master mode
//  <i>Enable HAL Driver Component
///*启用主机模式*/
#define SMODBUS_USING_MASTER
//  </c>

// <c1>Enable coil function in register pool
//  <i>Enable HAL Driver Component
/*启用线圈*/
#define SMODBUS_USING_COIL
// </c>

// <c1>Enable input coil function in register pool
//  <i>Enable HAL Driver Component
/*启用输入线圈*/
#define SMODBUS_USING_INPUT_COIL
// </c>

// <c1>Enable input register function in register pool
//  <i>Enable HAL Driver Component
/*启用输入寄存器*/
#define SMODBUS_USING_INPUT_REGISTER
// </c>

// <c1>Enable hold register function in register pool
//  <i>Enable HAL Driver Component
/*启用保持寄存器*/
#define SMODBUS_USING_HOLD_REGISTER
// </c>

// <<< end of configuration section >>>

/*包含对应操作系统接口:tool.h中包含对应api*/
#if (SMODBUS_USING_RTOS == 1)

#elif (SMODBUS_USING_RTOS == 2)

#else

#endif

/*检查关联寄存器组是否同时启用*/
#if defined(SMODBUS_USING_COIL) && !defined(SMODBUS_USING_INPUT_COIL)
#error Input coil not defined!
#elif !defined(SMODBUS_USING_COIL) && defined(SMODBUS_USING_INPUT_COIL)
#error Coil not defined!
#elif defined(SMODBUS_USING_INPUT_REGISTER) && !defined(SMODBUS_USING_HOLD_REGISTER)
#error Holding register not defined!
#elif !defined(SMODBUS_USING_INPUT_REGISTER) && defined(SMODBUS_USING_HOLD_REGISTER)
#error The input register is not defined!
#endif

#if (SMODBUS_USING_DEBUG == 1)
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "shell_port.h"
/*small modbus调试接口*/
#define SMODBUS_DEBUG(fmt, ...) shellPrint(shellGetCurrent(), fmt, ##__VA_ARGS__)
#define SMODBUS_DEBUG_D SMODBUS_DEBUG
#define SMODBUS_DEBUG_R SMODBUS_DEBUG
#elif (SMODBUS_USING_DEBUG == 2)
#undef DBG_TAG
#define DBG_TAG "smd"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#define SMODBUS_DEBUG_R dbg_raw
#define SMODBUS_DEBUG_D LOG_D
#define SMODBUS_DEBUG_I LOG_I
#define SMODBUS_DEBUG_W LOG_W
#define SMODBUS_DEBUG_E LOG_E
#else
#define SMODBUS_DEBUG
#endif

#ifdef __cplusplus
}
#endif
#endif /* PACKAGES_SMALL_MODBUS_INC_SMALL_MODBUS_CFG_H_ */
