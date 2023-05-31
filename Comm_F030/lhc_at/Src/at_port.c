#include "at.h"
#include "comdef.h"
#include "main.h"
#include "tool.h"
#include "lora.h"
#if defined(USING_RTTHREAD)
#include "rtthread.h"
#else
#include "cmsis_os.h"
#endif
#include "shell_port.h"

static AtHandle const At_Table[] = {
	{.Name = CMD_MODE, .pSend = "+++", "a", NULL},
	{.Name = CMD_SURE, .pSend = "a", AT_CMD_OK, NULL},
	{.Name = EXIT_CMD, .pSend = "AT+ENTM", NULL, NULL},									 /*退出命令模式，恢复原工作模式*/
	{.Name = SET_ECHO, .pSend = "AT+E=" SECHO, AT_CMD_OK, NULL},						 /*设置/查询模块 AT 命令回显设置*/
	{.Name = RESTART, .pSend = "AT+Z", "LoRa Start!", NULL},							 /*重启模块*/
	{.Name = RECOVERY, .pSend = "AT+CFGTF", "+CFGTF:SAVED", NULL},						 /*复制当前配置参数为用户默认出厂配置*/
	{.Name = SELECT_NID, .pSend = "AT+NID", "+NID:", NULL},								 /*查询模块节点 ID*/
	{.Name = SELECT_VER, .pSend = "AT+VER", "+VER:", NULL},								 /*查询模块固件版本*/
	{.Name = SET_UART, .pSend = "AT+UART=" BAUD_PARA, "+UART:" BAUD_PARA, NULL},		 /*设置串口参数*/
	{.Name = WORK_MODE, .pSend = "AT+WMODE=" SWORK_MODE, "+WMODE:" SWORK_MODE, NULL},	 /*设置工作模式*/
	{.Name = POWER_MODE, .pSend = "AT+PMODE=" SPOWER_MODE, "+PMODE:" SPOWER_MODE, NULL}, /*设置功耗模式*/
	{.Name = SET_TIDLE, .pSend = "AT+ITM=" SIMT, "+ITM:" SIMT, NULL},					 /*设置空闲时间:LR/LSR模式有效*/
	{.Name = SET_TWAKEUP, .pSend = "AT+WTM=" SWTM, "+WTM:" SWTM, NULL},					 /*设置唤醒间隔：此参数对 RUN、LSR 模式无效*/
	{.Name = SPEED_GRADE, .pSend = "AT+SPD=" SSPD, "+SPD:" SSPD, NULL},					 /*设置速率等级*/
	{.Name = TARGET_ADDR, .pSend = "AT+ADDR=" SADDR, "+ADDR:" SADDR, NULL},				 /*设置目的地址*/
	{.Name = CHANNEL, .pSend = "AT+CH=" SCH, "+CH:" SCH, NULL},							 /*设置信道*/
	{.Name = CHECK_ERROR, .pSend = "AT+FEC=" SFEC, "+FEC:" SFEC, NULL},					 /*设置前向纠错*/
	{.Name = TRANS_POWER, .pSend = "AT+PWR=" SPWR, "+PWR:" SPWR, NULL},					 /*设置发射功率*/
	{.Name = SET_OUTTIME, .pSend = "AT+RTO=" SRTO, "+RTO:" SRTO, NULL},					 /*设置接收超时时间*/
	{.Name = SIG_STREN, .pSend = "AT+SQT=" SSQT, "+SQT:" SSQT, NULL},					 /*查询信号强度/设置数据自动发送间隔*/
	// {.Name = SET_KEY, .pSend = "AT+KEY=" SKEY, "+KEY:" SKEY, NULL},                      /*设置数据加密字*/
	{.Name = LOW_PFLAG, .pSend = "AT+PFLAG=" SPFLAG, "+PFLAG:" SPFLAG, NULL},		/*设置/查询快速进入低功耗使能标志*/
	{.Name = LOW_PDATE, .pSend = "AT+PDATE=" SPDATE, "+PDATE:" SPDATE, NULL},		/*设置/查询快速进入低功耗数据*/
	{.Name = FINISH_FLAG, .pSend = "AT+SENDOK=" SSENDOK, "+SENDOK:" SSENDOK, NULL}, /*设置/查询发送完成回复标志*/
};
#define AT_TABLE_SIZE (sizeof(At_Table) / sizeof(AtHandle))

static const char *atText[] = {
	[CONF_MODE] = "Note: Enter configuration!\r\n",
	[FREE_MODE] = "Note: Enter free mode!\r\n",
	[UNKOWN_MODE] = "Error: Unknown mode!\r\n",
	[USER_ESC] = "Warning: User cancel!\r\n",
	[CONF_ERROR] = "Error: Configuration failed!\r\n",
	[CONF_TOMEOUT] = "Error: Configuration timeout.\r\n",
	[CONF_SUCCESS] = "Success: Configuration succeeded!\r\n",
	[INPUT_ERROR] = "Error: Input error!\r\n",
	[CMD_MODE] = "Note: Enter transparent mode!\r\n",
	[CMD_SURE] = "Note: Confirm to exit the transparent transmission mode?\r\n",
	[SET_ECHO] = "Note: Set echo?\r\n",
	[SET_UART] = "Note: Set serial port parameters!\r\n",
	[WORK_MODE] = "Note: Please enter the working mode?(0:TRANS/1:FP)\r\n",
	[POWER_MODE] = "Note: Please enter the power consumption mode?(0:RUN/1:LR/2:WU/3:LSR)\r\n",
	[SET_TIDLE] = "Note: Set idle time.\r\n",
	[SET_TWAKEUP] = "Note: Set wake-up interval.\r\n",
	[SPEED_GRADE] = "Note: Please enter the rate level?(1~10)\r\n",
	[TARGET_ADDR] = "Note: Please enter the destination address?(0~65535)\r\n",
	[CHANNEL] = "Note: Please enter the channel?(0~127)\r\n",
	[CHECK_ERROR] = "Note: Enable forward error correction?(1:true/0:false)\r\n",
	[TRANS_POWER] = "Note: Please input the transmission power?(10~20db)\r\n",
	[SET_OUTTIME] = "Note: Please enter the receiving timeout?(LR/LSR mode is valid,0~15000ms)\r\n",
	// [SET_KEY] = "Note: Please enter the data encryption word?(16bit Hex)\r\n",
	[RESTART] = "Note: Device restart!\r\n",
	[SIG_STREN] = "Note: Query signal strength.\r\n",
	[EXIT_CMD] = "Note: Exit command mode!\r\n",
	[RECOVERY] = "Note: Restore default parameters!\r\n",
	[SELECT_NID] = "Note: Query node ID?\r\n",
	[SELECT_VER] = "Note: Query version number?\r\n",
	[LOW_PFLAG] = "Note: Set / query fast access low power enable flag.\r\n",
	[LOW_PDATE] = "Note: Set / query fast access to low-power data.\r\n",
	[FINISH_FLAG] = "Note: Set / query sending completion reply flag.\r\n",
	[EXIT_CONF] = "Note: Please press \"ESC\" to end the configuration!\r\n",
	[NO_CMD] = "Error: Command does not exist!\r\n",
};

/**
 * @brief  获取at缓冲区中字节数
 * @param  None
 * @retval 字符个数
 */
static unsigned int at_get_count(void)
{
	pLoraHandle pl = Lora_Object;

	if (pl->Uart.rx.count > pl->Uart.rx.size) // 截断缓冲区，否则上层可能溢出
	{
		at_printf("@error: recv buf overflow.\r\n");
		pl->Uart.rx.count = pl->Uart.rx.size - 1;
	}

	return pl ? (pl->Uart.rx.count) : 0;
}

/**
 * @brief  清空at接收缓冲区
 * @param  None
 * @retval None
 */
static void at_reset_recv_buf(void)
{
	pLoraHandle pl = Lora_Object;

	if (pl && pl->Uart.rx.pbuf)
	{
		memset(pl->Uart.rx.pbuf, 0, pl->Uart.rx.size);
		pl->Uart.rx.count = 0;
	}
}

/**
 * @brief  获取at缓冲区指针
 * @param  None
 * @retval 目标指针
 */
static const char *at_get_recv_buf(void)
{
	pLoraHandle pl = Lora_Object;

	return ((pl && pl->Uart.rx.pbuf)
				? ((const char *)pl->Uart.rx.pbuf)
				: NULL);
}

/**
 * @brief  获取at缓冲区指针
 * @param  cmd 目标指令
 * @param  len 字符串长度
 * @retval 目标指针
 */
static void at_send_cmd(char *cmd, unsigned short len)
{
	pLoraHandle pl = Lora_Object;

	if (NULL == pl || !len)
		return;

	pl->Lora_Send(pl, (uint8_t *)cmd, len);
}

/**
 * @brief  获取at数据接收完成标志
 * @param  cmd 目标指令
 * @param  len 字符串长度
 * @retval 目标指针
 */
static uint8_t at_get_resp_flag(unsigned short timeout)
{
	// extern osSemaphoreId At_ReciveHandle;
	// if (osOK == osSemaphoreWait(At_ReciveHandle, timeout))
	// {
	//     at_delay(250);
	//     return true;
	// }
	// else
	//     return false;

	pLoraHandle pl = Lora_Object;

	timeout = timeout < AT_BASE_TIMEOUT
				  ? AT_BASE_TIMEOUT
				  : timeout / AT_BASE_TIMEOUT;
	while (!pl->Uart.rx.count && timeout--)
	{
		at_delay(AT_BASE_TIMEOUT);
	}

	if (pl->Uart.rx.count)
		return true;
	return false;
}

struct at_basic_info at_object = {
	.cmd = {
		.table = NULL,
		.size = AT_TABLE_SIZE,
	},
	.note = {
		.str = NULL,
		.size = AT_TABLE_SIZE,
	},
	.at_get_count = at_get_count,
	.at_reset_recv_buf = at_reset_recv_buf,
	.at_get_recv_buf = at_get_recv_buf,
	.at_send_cmd = at_send_cmd,
	.at_get_resp_flag = at_get_resp_flag,
};

// void at_task(void const * argument)
// {
//     pLoraHandle pl = Lora_Object;
//     uint8_t cmd = *((uint8_t *)argument);
//     for (;;)
//     {
//         if (pl->Cs.pGPIOx)
//             HAL_GPIO_WritePin(pl->Cs.pGPIOx, pl->Cs.Gpio_Pin, GPIO_PIN_RESET);
//         cmd ? at_free_model() : at_config_model();
//         /*禁止接收引脚*/
//         if (pl->Cs.pGPIOx)
//             HAL_GPIO_WritePin(pl->Cs.pGPIOx, pl->Cs.Gpio_Pin, GPIO_PIN_SET);

//         (AT_SHELL)->read = at_read; //read函数还原
//         pl->Mode = Lora_Nomal_Recive;
//         vTaskDelete(NULL);
//     }
// }

/**
 * @brief  通过AT指令配置L101模块参数
 * @param  cmd 命令模式 1参数配置 2自由指令
 * @retval None
 */
void at_shell(unsigned char cmd)
{
	Shell *sh = shellGetCurrent();
	pLoraHandle pl = Lora_Object;

	if (NULL == pl || cmd > FREE_MODE)
	{
		at_wstring(atText[UNKOWN_MODE]);
		return;
	}
	at_wstring(atText[cmd]);

	at_object.cmd.table = (pAtHandle)At_Table;
	at_object.note.str = (char **)atText;
	at_object.at_read = sh->read; // read函数转移

	pl->Mode = Lora_Param_Cfg;
	sh->read = NULL;
	/*使能接收引脚*/
	if (pl->Cs.pGPIOx)
		HAL_GPIO_WritePin(pl->Cs.pGPIOx, pl->Cs.Gpio_Pin, GPIO_PIN_RESET);
	cmd ? at_free_model(&at_object) : at_config_model(&at_object);
	/*禁止接收引脚*/
	if (pl->Cs.pGPIOx)
		HAL_GPIO_WritePin(pl->Cs.pGPIOx, pl->Cs.Gpio_Pin, GPIO_PIN_SET);
	sh->read = at_object.at_read; // read函数还原
	pl->Mode = Lora_Nomal_Recive;

#if defined(USING_DEBUG)
	// shellPrint(sh, "portNVIC_INT_CTRL_REG = 0x%x\r\n", portNVIC_INT_CTRL_REG);
#endif

	// osThreadDef(at, at_task, osPriorityNormal, 0, 256);
	// if (NULL == osThreadCreate(osThread(at), &cmd))
	// {
	//     at_printf("at task create fail.\r\n");
	//     sh->read = at_read; //read函数还原
	//     pl->Mode = Lora_Nomal_Recive;
	// }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
				 at, at_shell, at module operate.);
