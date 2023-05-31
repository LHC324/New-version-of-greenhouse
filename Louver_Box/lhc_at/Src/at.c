#include <stdbool.h>
#include <string.h>
#include "at.h"

#ifndef NULL
#define NULL ((void *)0)
#endif
#if (1 == AT_USING_RTOS)
#include "cmsis_os.h"
#endif
#if AT_USING_SHELL
#include "shell_port.h"
#endif

/*清除HAL库计数器*/
#define SET_HAL_TICK(__value) (uwTick = __value)

/**
 * @brief       等待接收到指定串
 * @param[in]   pt      at对象
 * @param[in]   resp    - 期待待接收串(如"OK",">")
 * @param[in]   timeout - 等待超时时间
 */
At_InfoList at_wait_recv_str(struct at_basic_info const *const pt,
                             const char *resp,
                             unsigned short timeout)
{
    if (NULL == pt->at_get_count ||
        NULL == pt->at_get_resp_flag ||
        NULL == pt->at_get_recv_buf)
        return CONF_ERROR;

    At_InfoList ret = CONF_TOMEOUT;
    char *buf = (char *)pt->at_get_recv_buf();

    if (NULL == buf)
        return CONF_ERROR;

    if (!pt->at_get_resp_flag(timeout))
        goto __exit;

    buf[pt->at_get_count()] = '\0';
    ret = strstr((const char *)(buf), resp)
              ? CONF_SUCCESS
              : (strstr((const char *)(buf), AT_CMD_ERROR)
                     ? CONF_ERROR
                     : CONF_TOMEOUT);
__exit:
    at_printf(">[L102->MCU]:%s\r\n", buf);
    pt->at_reset_recv_buf();

    return ret;
}

/**
 * @brief  获取目标at指令
 * @param[in] pt at对象
 * @retval None
 */
AtHandle const *at_get_cmd(AtHandle const *pt,
                           At_InfoList list,
                           unsigned int size)
{
    unsigned short i = 0;

    while (i < size)
    {
        if (pt[i].Name == list)
        {
            return &pt[i];
        }
        i++;
    }

    return NULL;
}

/**
 * @brief	AT模块接收上层自由指令
 * @details
 * @param[in] pt at对象
 * @retval	None
 */
static void at_exec_cmd(struct at_basic_info const *const pt,
                        AtHandle const *pat,
                        char *const psource)
{
    if (NULL == pt)
        return;

    AtHandle const *ps = NULL;
    At_InfoList result = CONF_SUCCESS;
    char *pre = NULL, *pdest = NULL;

    /*进入命令模式，并关闭回显*/
    for (At_InfoList at_cmd = CMD_MODE; at_cmd < POWER_MODE; at_cmd++)
    { /*执行完毕后退出指令模式*/
        at_cmd = (at_cmd == POWER_MODE - 1U) ? RESTART : at_cmd;
        ps = at_get_cmd(pat, at_cmd, pt->cmd.size);
        pre = ((at_cmd == CMD_MODE) || (at_cmd == SET_ECHO))
                  ? ps->pRecv
                  : AT_CMD_OK;
        if (ps)
        {
#if (1 == AT_USING_RTOS)
            char *pStr = (char *)at_malloc(sizeof(char) * strlen(ps->pSend) + strlen(AT_CMD_END_MARK_CRLF));
            if (pStr)
            { /*拷贝时带上'\0'*/
                memcpy(pStr, ps->pSend, strlen(ps->pSend) + 1U);
                if (at_cmd > CMD_SURE)
                {
                    strcat(pStr, AT_CMD_END_MARK_CRLF);
                }
                pdest = (at_cmd != SET_UART) ? pStr : (char *)psource;
                if (at_cmd < pt->note.size)
                    at_printf("\r\n%s", pt->note.str[at_cmd]);
                at_printf(">[MCU->L102]:%s\r\n", pdest);
                if (pt->at_send_cmd)
                    pt->at_send_cmd(pdest, strlen(pdest));
            }
            at_free(pStr);
#else
#endif
        }
        result = at_wait_recv_str(pt, pre, MAX_URC_RECV_TIMEOUT);
        at_wstring(pt->note.str[result]);
        if ((result != CONF_SUCCESS) || (at_cmd == RESTART))
        {
            at_wstring(pt->note.str[EXIT_CONF]);
        }
    }
}

/**
 * @brief  自由模式
 * @note   关于多次运行后：接收错误、从中断中进入临界区：
 *         https://www.freertos.org/FreeRTOS_Support_Forum_Archive/August_2006/freertos_portENTER_critical_1560359.html
 * @param  pt at对象
 * @retval None
 */
void at_free_model(struct at_basic_info const *const pt)
{
    if (NULL == pt || NULL == pt->cmd.table || NULL == pt->note.str)
        return;

    char recv_data = '\0';
    unsigned short len = 0;
#if (1 == AT_USING_RTOS)
#define AT_BUF_SIZE 64U
    char *pbuf = (char *)at_malloc(sizeof(char) * AT_BUF_SIZE);
    if (!pbuf)
        goto __exit;
#else
    char pbuf[64U] = {0};
#endif

#if defined(USING_DEBUG)
        // at_printf( "pH = 0x%02x\r\n", pH);
#endif
    while (1)
    {
        if (NULL == pt->at_read)
            break;

        if (!pt->at_read(&recv_data, 0x01))
            continue;

        switch (recv_data)
        {
        case AT_ENTER_CODE:
        {
            pbuf[len] = '\0';
            len = 0U;
            at_printf("\r\nInput:%s\r\n", pbuf);
            if (pt->at_reset_recv_buf) // 清空上一次缓存“LoRa Start!”
                pt->at_reset_recv_buf();
            at_exec_cmd(pt, pt->cmd.table, strcat(pbuf, AT_CMD_END_MARK_CRLF));
            len = 0U;
        }
        break;
        case AT_BACKSPACE_CODE:
        {
            len = len ? at_backspace(0x01), --len : 0;
        }
        break;
        case AT_ESC_CODE:
        {
            goto __exit;
        }
        default:
        {
#if defined(USING_DEBUG)
            // at_printf( "\r\nlen = %d\r\n", len);
#endif
            pbuf[len++] = recv_data;
            // at_printf( "\r\nval = %c\r\n", pbuf[len]);
            len = (len < AT_BUF_SIZE) ? len : 0U;
            at_wchar(&recv_data, 0x01);
        }
        break;
        }
    }

__exit:
#if (1 == AT_USING_RTOS)
    at_free(pbuf);
#endif
}

/**
 * @brief  参数配置模式
 * @param  pt at对象
 * @retval None
 */
void at_config_model(struct at_basic_info const *const pt)
{
    if (NULL == pt || NULL == pt->cmd.table || NULL == pt->note.str)
        return;

    char recv_data = '\0';
    AtHandle const *pAt = pt->cmd.table, *pS = NULL;
    char *pRe = NULL;
    At_InfoList result = CONF_SUCCESS;
    bool at_mutex = false;

    while (recv_data != AT_ESC_CODE)
    {
        if (NULL == pt->at_read)
            break;
        /*接收到的数据不为0*/
        if (!pt->at_read(&recv_data, 0x01))
            continue;

#if defined(USING_DEBUG)
            // at_printf( "*pdata = 0x%02x\r\n", *pData);
#endif
        for (At_InfoList at_cmd = CMD_MODE; (at_cmd < SIG_STREN) && (!at_mutex); at_cmd++)
        {
            pS = at_get_cmd(pAt, at_cmd, pt->cmd.size);
            if (at_cmd < pt->note.size)
                at_printf("\r\n%s", pt->note.str[at_cmd]);
            at_printf(">[MCU->L102]:%s\r\n", pS->pSend);
            if (pS->pSend)
            {
#if (1 == AT_USING_RTOS)
                char *pStr = (char *)at_malloc(sizeof(char) * strlen(pS->pSend) + strlen(AT_CMD_END_MARK_CRLF));
                if (pStr)
                { /*拷贝时带上'\0'*/
                    memcpy(pStr, pS->pSend, strlen(pS->pSend) + 1U);
                    if (at_cmd > CMD_SURE)
                    {
                        strcat(pStr, AT_CMD_END_MARK_CRLF);
                    }
                    if (pt->at_send_cmd)
                        pt->at_send_cmd(pStr, strlen(pStr));
                }
                at_free(pStr);
#else
                // pH->mdRTUSendString(pH, (uint8_t *)pS->pSend, strlen(pS->pSend));
                // /*加上结尾*/
                // if (at_cmd > CMD_SURE)
                // {
                //     pH->mdRTUSendString(pH, (uint8_t *)AT_CMD_END_MARK_CRLF, strlen(AT_CMD_END_MARK_CRLF));
                // }
#endif
            }
            else
            {
                at_mutex = true;
                at_wstring(pt->note.str[NO_CMD]);
                break;
            }
            pRe = ((at_cmd == CMD_MODE) || (at_cmd == SET_ECHO))
                      ? pS->pRecv
                      : AT_CMD_OK;
            result = at_wait_recv_str(pt, pRe, MAX_URC_RECV_TIMEOUT);
            at_wstring(pt->note.str[result]);
            if ((result != CONF_SUCCESS) || (at_cmd == RESTART))
            {
                at_mutex = true;
                at_wstring(pt->note.str[EXIT_CONF]);
                break;
            }
        }
    }

    if (pt->at_reset_recv_buf) // 清空上一次缓存“LoRa Start!”
        pt->at_reset_recv_buf();
}
