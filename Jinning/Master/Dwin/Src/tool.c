#include "tool.h"
#include "shell_port.h"
#include <stdlib.h>

unsigned int g_Debug_Bits = 0;

/*定义从机中断表*/
IRQ_Request Re_IRQ[CARD_NUM_MAX] = {
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
    {.site = INACTIVE_SITE, .Priority = PRIORITY_MAX, .flag = false},
};
IRQ_Code IRQ[CARD_NUM_MAX] = {
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
    {.SlaveId = CARD_NUM_MAX, .Priority = PRIORITY_MAX, .TypeCoding = Card_None, .Number = 0},
};
#if (USING_FREERTOS_LIST)
List_t Ready_List, Block_List;
#endif

Slave_IRQTableTypeDef IRQ_Table = {
    .pIRQ = IRQ,
    .TableCount = 0,
    // .Amount = 0,
    .pReIRQ = Re_IRQ,
    .SiteCount = 0,
    // .LReady = &Ready_List,
    // .LBlock = &Block_List,
};

#define SD sizeof(IRQ)

#if (USING_FREERTOS_LIST)
/**
 * @brief	添加列表项到列表
 * @details
 * @param	None
 * @retval	None
 */
void Add_ListItem(List_t *list, TickType_t data)
{
    ListItem_t *p = NULL;
    p = (ListItem_t *)pvPortMalloc(sizeof(ListItem_t));
    if (p == NULL)
    {
        vPortFree(p);
        return;
    }
    vListInitialiseItem(p);
    listSET_LIST_ITEM_VALUE(p, data);
    /*按照data大小升序排列*/
    // vListInsert(list, p);
    vListInsertEnd(list, p);
}
/**
 * @brief	从列表中查找指定列表项
 * @details
 * @param	None
 * @retval	None
 */
ListItem_t *Find_ListItem(List_t *list, TickType_t data)
{
    ListItem_t *p = list->xListEnd.pxNext;

    for (; p->xItemValue != portMAX_DELAY; p = p->pxNext)
    {
        if (listGET_LIST_ITEM_VALUE(p) == data)
        {
            return p;
        }
    }
    return NULL;
}

/**
 * @brief	从列表中移除列表
 * @details
 * @param	None
 * @retval	列表中剩余的列表项
 */
UBaseType_t Remove_ListItem(List_t *list, TickType_t data)
{
    UBaseType_t items;
    /*先从对应列表中查找到列表项，在移除*/
    ListItem_t *p = Find_ListItem(list, data);
    if (p && (p->xItemValue != portMAX_DELAY))
    {
        items = uxListRemove(p);
        /*释放链表节点*/
        vPortFree(p);
        return items;
    }
    return 0;
}

/**
 * @brief	获取一个列表项
 * @details
 * @param	None
 * @retval	目标列表项
 */
ListItem_t *Get_OneListItem(List_t *list, ListItem_t **p)
{
    // static ListItem_t *p = NULL;

    if (list == NULL)
    {
        return NULL;
    }
    *p = *p ? (*p) : ((ListItem_t *)&list->xListEnd);
    *p = ((*p)->pxNext->xItemValue == portMAX_DELAY) ? ((ListItem_t *)list->xListEnd.pxNext) : ((*p)->pxNext);

    return *p;
}
#endif

// Range new_Range(int s, int e)
// {
//     Range r;
//     r.start = s;
//     r.end = e;
//     return r;
// }

#if (USING_SWAP_ANY)
/**
 * @brief	任意元素交换
 * @details
 * @param	__type 数据类型
 * @param   __lhs  带交换左边数据r
 * @param   __rhs  带交换右边数据
 * @retval	None
 */
#define SWAP(__type, __lhs, __rhs) \
    do                             \
    {                              \
        __type temp = __lhs;       \
        __lhs = __rhs;             \
        __rhs = temp;              \
    } while (false)
#else
/*使用数组元素时，下标不能相同; 且限制定于整形数据交换*/
#define SWAP(__A, __B)  \
    do                  \
    {                   \
        (__A) ^= (__B); \
        (__B) ^= (__A); \
        (__A) ^= (__B); \
    } while (0)
#endif

/**
 * @brief	非递归快速排序
 * @details
 * @param	pData 目标排序数据源
 * @param   len   数据源尺寸
 * @retval	None
 */
void Quick_Sort(QUICK_SORT_TYPE *pData, int len)
{
    int site = 0;
    /*避免len等于负值时错误 */
    if ((len <= 0) || (pData == NULL))
        return;
    /*pStack模拟堆疊,site为数量,pStack[site++]为push,pStack[--site]为pop且取得元素*/
    Range *pStack = (Range *)CUSTOM_MALLOC(sizeof(Range) * len);
    if (!pStack)
    {
        goto __exit;
    }

    pStack[site].start = 0, pStack[site++].end = len - 1;
    while (site)
    {
        Range range = pStack[--site];
        if (range.start >= range.end)
            continue;
        int mid = pData[range.end].Priority;
        int left = range.start, right = range.end - 1;
        while (left < right)
        {
            while ((pData[left].Priority < mid) && (left < right))
                left++;
            while ((pData[right].Priority >= mid) && (left < right))
                right--;
            if (left != right)
            {
                SWAP(QUICK_SORT_TYPE, pData[left], pData[right]);
            }
        }
        if ((pData[left].Priority >= pData[range.end].Priority) && (left != range.end))
        {
            SWAP(QUICK_SORT_TYPE, pData[left], pData[range.end]);
        }
        else
        {
            left++;
        }

        pStack[site].start = range.start, pStack[site++].end = left - 1;
        pStack[site].start = left + 1, pStack[site++].end = range.end;
    }
__exit:
    CUSTOM_FREE(pStack);
}

/**
 * @brief	非递归快速排序（按照id号）
 * @details
 * @param	pData 目标排序数据源
 * @param   len   数据源尺寸
 * @retval	None
 */
void Quick_Sort_Id(IRQ_Code *pData, int len)
{
    int site = 0;
    /*避免len等于负值时错误 */
    if ((len <= 0) || (pData == NULL))
        return;
    /*pStack模拟堆疊,site为数量,pStack[site++]为push,pStack[--site]为pop且取得元素*/
    Range *pStack = (Range *)CUSTOM_MALLOC(sizeof(Range) * len);
    if (!pStack)
    {
        goto __exit;
    }

    pStack[site].start = 0, pStack[site++].end = len - 1;
    while (site)
    {
        Range range = pStack[--site];
        if (range.start >= range.end)
            continue;
        int mid = pData[range.end].SlaveId;
        int left = range.start, right = range.end - 1;
        while (left < right)
        {
            while ((pData[left].SlaveId < mid) && (left < right))
                left++;
            while ((pData[right].SlaveId >= mid) && (left < right))
                right--;
            if (left != right)
            {
                SWAP(IRQ_Code, pData[left], pData[right]);
            }
        }
        if ((pData[left].SlaveId >= pData[range.end].SlaveId) && (left != range.end))
        {
            SWAP(IRQ_Code, pData[left], pData[range.end]);
        }
        else
        {
            left++;
        }

        pStack[site].start = range.start, pStack[site++].end = left - 1;
        pStack[site].start = left + 1, pStack[site++].end = range.end;
    }
__exit:
    CUSTOM_FREE(pStack);
}

/**
 * @brief  在编码阶段检测是否出现相同的从机
 * @param  irq 中断表句柄
 * @param  target_id 目标从机id
 * @retval None
 */
static bool Find_SameId(Slave_IRQTableTypeDef *irq, uint8_t target_id)
{
    for (IRQ_Code *p = irq->pIRQ; p < irq->pIRQ + irq->TableCount; p++)
    {
        if (p->SlaveId == target_id)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief  通过从机位置从中断表中获取目标句柄
 * @note   此处查找的目标对象唯一
 * @param  irq 中断表句柄
 * @param  target_id 目标从机id
 * @retval None
 */
IRQ_Code *Find_TargetSlave_AdoptId(Slave_IRQTableTypeDef *irq, uint8_t target_id)
{
    for (IRQ_Code *p = irq->pIRQ; p < irq->pIRQ + irq->TableCount; p++)
    {
        if (p->SlaveId == target_id)
        {
            return p;
        }
    }
    return NULL;
}

/**
 * @brief  通过从机类型从中断表中获取目标句柄
 * @note   注意此处查找的是不唯一对象
 * @param  irq 中断表句柄
 * @param  p_current 当前从机指针
 * @retval None
 */
IRQ_Code *Find_TargetSlave_AdoptType(Slave_IRQTableTypeDef *irq, IRQ_Code *p_current)
{
    IRQ_Code *p_target = NULL;
    bool first_flag = false;

    // for (IRQ_Code *p = irq->pIRQ; p < irq->pIRQ + irq->TableCount; p++)
    for (IRQ_Code *p = irq->pIRQ; p < p_current; p++)
    {
        if (p->TypeCoding == p_current->TypeCoding)
        {
            /*始终找到id号最大的同类型板卡*/
            if (!first_flag)
            {
                first_flag = true;
                p_target = p;
            }
            else
            {
                if (p->SlaveId > p_target->SlaveId)
                {
                    p_target = p;
                }
            }
        }
    }
    return p_target;
}

/**
 * @brief  通过板卡类型查找目标从机的slaveid存储到指定位置
 * @note   此处查找的目标对象不唯一;保障从机编号的有序
 * @param  irq 中断表句柄
 * @param  type 目标从机id
 * @param  psave 存储位置
 * @param  nums  需要查找的目标板卡数量
 * @retval false/true
 */
bool Save_TargetSlave_Id(Slave_IRQTableTypeDef *irq,
                         Card_Tyte type,
                         R_TargetTypeDef *psave,
                         uint8_t nums)
{
    bool ret = false;
    uint8_t *q = psave->p_id;
    /*Find target board*/
    for (IRQ_Code *p = irq->pIRQ; p < irq->pIRQ + irq->TableCount; p++)
    {
        if ((p->TypeCoding == type) && psave && q < psave->p_id + psave->id_size)
        {
            if (nums && (psave->count++ < nums))
            {
                *q++ = p->SlaveId;
                ret = true;
            }
            else
                break;
        }
    }
    /*按照从机id升序排列*/
    // if (psave->count)
    // {
    //     for (uint8_t i = 0; i < psave->count; i++)
    //     {
    //         if (psave->p_id[i] > psave->p_id[i + 1])
    //         {
    //             SWAP(uint8_t, psave->p_id[i], psave->p_id[i + 1]);
    //         }
    //     }
    // }
    return ret;
}

/*获取板卡中断偏移号*/
#define Get_Card_IRQOffset(__type)                      \
    ((__type) == Card_AnalogInput                       \
         ? 0x00                                         \
         : ((__type) == Card_DigitalInput               \
                ? 0x10                                  \
                : ((__type) == Card_DigitalOutput       \
                       ? 0x20                           \
                       : ((__type) == Card_AnalogOutput \
                              ? 0x30                    \
                              : 0x40))))
/**
 * @brief  进行板卡中断编码
 * @param  irq 中断表句柄
 * @retval None
 */
void IRQ_Coding(Slave_IRQTableTypeDef *irq, uint8_t code)
{
    IRQ_Code *p_current = &irq->pIRQ[irq->TableCount];
    // IRQ_Code *p_current = NULL;
    uint8_t temp_slaveid = code & 0x0F;
    Card_Tyte temp_typecoding = (Card_Tyte)(code & 0xF0);

    if (irq->TableCount < CARD_NUM_MAX)
    {
        // p_current = &irq->pIRQ[temp_slaveid];
        if (Find_SameId(irq, temp_slaveid))
        {
#if defined(USING_DEBUG)
            shellPrint(Shell_Object, "Error: Duplicate slave ID %d.\r\n", temp_slaveid);
#endif
            return;
        }
        if (temp_typecoding == Card_None)
        {
#if defined(USING_DEBUG)
            shellPrint(Shell_Object, "Error: Invalid board = %d.\r\n", temp_typecoding);
#endif
            return;
        }
#if defined(USING_DEBUG)
// shellPrint(Shell_Object, "p_current = 0x%p.\r\n", p_current);
#endif
        p_current->SlaveId = temp_slaveid;
        p_current->TypeCoding = temp_typecoding;

        IRQ_Code *p_target = Find_TargetSlave_AdoptType(irq, p_current);
        /*中断表中已经存在同类型板卡:ID不同，但是类型相同*/
        if (p_target)
        {
            p_current->Priority = p_target->Priority + 1U;
            /*当前板卡是同类型中第几张板卡*/
            p_current->Number = p_target->Number + 1U;
            /*保障从机号在前相同板卡，计数值在前*/
            if (p_target->SlaveId > p_current->SlaveId)
            {
                // p->Number = p_current->Number;
                SWAP(uint16_t, p_target->Number, p_current->Number);
            }
        }
        /*首次加入*/
        else
        {
            p_current->Number = 0;
            p_current->Priority = Get_Card_IRQOffset(p_current->TypeCoding);
#if defined(USING_DEBUG)
            shellPrint(Shell_Object, "TypeCoding %#x first Join, Priority is %#x.\r\n",
                       p_current->TypeCoding, p_current->Priority);
#endif
        }
#if (USING_FREERTOS_LIST)
        if (irq->LReady)
        {
            /*把优先级加入就绪列表*/
            Add_ListItem(irq->LReady, p_current->Priority);
        }
#endif
        if (irq->pIRQ && (irq->TableCount++ > 1))
        {
            // Quick_Sort(irq->pIRQ, irq->TableCount);
            Quick_Sort_Id(irq->pIRQ, irq->TableCount);
        }
        /*标记当前板卡优先级设置完成*/
        // p_current->flag = false;
        // irq->TableCount++;
    }
    else
    {
#if defined(USING_DEBUG)
        shellPrint(Shell_Object, "Error: Card slot full!\r\n");
#endif
    }
}

/**
 * @brief  按照板卡类型查找优先级最高的从机设备
 * @param  irq 中断表句柄
 * @param  slave_id 从机id
 * @retval 从机板卡类型
 */
Card_Tyte Find_HighP_Device(Slave_IRQTableTypeDef *irq, IRQ_Request *rp_irq)
{
    IRQ_Code *p_current = &irq->pIRQ[rp_irq->site];
    uint16_t min_priority = PRIORITY_MAX;

    /*从当处理位置开始查找*/
    for (IRQ_Code *p = p_current; p < p_current + irq->TableCount; p++)
    { /*把中断优先级表升序排列*/
        // IRQ_Code current_Iraq = *p;
        min_priority = min_priority <= p->Priority ? min_priority : p->Priority;
    }

    return irq->pIRQ[rp_irq->site].TypeCoding;
}

/**
 * @brief  DMA+空闲中断接收不定长数据
 * @param  pd 接收数据对象句柄
 * @retval None
 */
void DMA_Recive(pDmaHandle pd)
{
    /*Gets the idle flag so that the idle flag is set*/
    if ((__HAL_UART_GET_FLAG(pd->huart, UART_FLAG_IDLE) != RESET))
    {
        /*Clear idle interrupt flag*/
        __HAL_UART_CLEAR_IDLEFLAG(pd->huart);
        if (pd && (pd->pRbuf) && (pd->phdma))
        {
            /*Stop DMA transmission to prevent busy receiving data and interference during data processing*/
            HAL_UART_DMAStop(pd->huart);
            /*Get the number of untransmitted data in DMA*/
            /*Number received = buffersize - the number of data units remaining in the current DMA channel transmission */
            if (pd->pRxCount)
                *(pd->pRxCount) = pd->RxSize - __HAL_DMA_GET_COUNTER(pd->phdma);
            /*Reopen DMA reception*/
            HAL_UART_Receive_DMA(pd->huart, pd->pRbuf, pd->RxSize);
        }
#if defined(USING_FREERTOS)
        /*After opening the serial port interrupt, the semaphore has not been created*/
        if (pd->Semaphore)
        {
            /*Notification task processing*/
            osSemaphoreRelease(pd->Semaphore);
        }
#else
        pd->Recive_FinishFlag = true;
#endif
    }
}

/**
 * @brief  取得16bitCRC校验码
 * @param  ptr   当前数据串指针
 * @param  length  数据长度
 * @param  init_dat 校验所用的初始数据
 * @retval 16bit校验码
 */
uint16_t Get_Crc16(uint8_t *ptr, uint16_t length, uint16_t init_dat)
{
    uint16_t crc16 = init_dat;

    for (uint16_t i = 0; i < length; i++)
    {
        crc16 ^= *ptr++;

        for (uint16_t j = 0; j < 8; j++)
        {
            crc16 = (crc16 & 0x0001) ? ((crc16 >> 1) ^ 0xa001) : (crc16 >> 1U);
        }
    }
    return (crc16);
}

/**
 * @brief  大小端数据类型交换
 * @note   对于一个单精度浮点数的交换仅仅需要2次
 * @param  pData 数据
 * @param  start 开始位置
 * @param  length  数据长度
 * @retval None
 */
void Endian_Swap(uint8_t *pData, uint8_t start, uint8_t length)
{
    uint8_t i = 0;
    uint8_t tmp = 0;
    uint8_t count = length / 2U;
    uint8_t end = start + length - 1U;

    for (; i < count; i++)
    {
        tmp = pData[start + i];
        pData[start + i] = pData[end - i];
        pData[end - i] = tmp;
    }
}

#if (TOOOL_USING_LINUX_STAMP == 1)
/*
 *  Beijing time to linux timestamp
 */
unsigned int std_time_to_linux_stamp(rtc_t *prtc)
{
    if (prtc == NULL)
        return 0;

    unsigned short monthdays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unsigned short year = prtc->date.year + 2000;
    unsigned int linux_stamp = (year - 1970) * 365 * 24 * 3600 +
                               (monthdays[prtc->date.month - 1] +
                                prtc->date.date - 1) *
                                   24 * 3600 +
                               (prtc->time.hours - 8) * 3600 + prtc->time.minutes * 60 + prtc->time.seconds;

    linux_stamp += (prtc->date.month > 2 && (year % 4 == 0) &&
                    (year % 100 != 0 || year % 400 == 0)) *
                   24 * 3600; // 闰月

    year -= 1970;
    linux_stamp += (year / 4 - year / 100 + year / 400) * 24 * 3600; // 闰年

    return linux_stamp;
}

/*
 * Judge leap year and average year
 */
static unsigned char is_leap_year(unsigned short year)
{
    return ((((year) % 4 == 0 &&
              (year) % 100 != 0) ||
             (year) % 400 == 0));
}

/*
 *  Function function: get the week according to the specific date
 */
void get_weekday(rtc_t *prtc)
{
    unsigned short year = 0;
    unsigned char month = 0;

    // if (prtc->date.month == 1 || prtc->date.month == 2)
    if (prtc->date.month < 3)
    {
        year = prtc->date.year - 1 + 2000;
        month = prtc->date.month + 12;
    }
    else
    {
        year = prtc->date.year + 2000;
        month = prtc->date.month;
    }

    prtc->date.weelday = ((prtc->date.date + 2 * month + 3 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400) % 7) + 1;
}

/*
 *    Linux Time Stamp to std time
 */
void linux_stamp_to_std_time(rtc_t *prtc, unsigned int cur_stamp, int time_zone)
{
    unsigned short year = 1970;
    unsigned int counter = 0, count_temp; // 随着年份迭加，Counter记录￿??1970 ￿?? 1 ￿?? 1 日（00:00:00 GMT）到累加到的年份的最后一天的秒数
    unsigned char month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    cur_stamp += time_zone * 3600; // 修正时区

    while (counter <= cur_stamp) // 假设今天￿??2018年某￿??天，则时间戳应小于等￿??1970-1-1 0:0:0 ￿?? 2018-12-31 23:59:59的�?�秒￿??
    {
        count_temp = counter; // CounterTemp记录完全1970-1-1 0:0:0 ￿?? 2017-12-31 23:59:59的�?�秒数后￿??出循￿??
        counter += 31536000;  // 加上今年（平年）的秒￿??

        if (is_leap_year(year))
        {
            counter += 86400; // 闰年多加￿??￿??
        }
        year++;
    }

    prtc->date.year = year - 1 - 2000; // 跳出循环即表示到达计数�?�当前年
    month[1] = (is_leap_year(year - 1) ? 29 : 28);
    counter = cur_stamp - count_temp; // counter = cur_stamp - count_temp  记录2018年已走的总秒￿??
    count_temp = counter / 86400;     // count_temp = counter/(24*3600)  记录2018年已【过去�?�天￿??
    counter -= count_temp * 86400;    // 记录今天已走的�?�秒￿??
    prtc->time.hours = counter / 3600;
    prtc->time.minutes = counter % 3600 / 60;
    prtc->time.seconds = counter % 60;

    for (unsigned char i = 0; i < 12; i++)
    {
        if (count_temp < month[i]) // 不能包含相等的情况，相等会导致最后一天切换到下一个月第一天时
        {
            // （即CounterTemp已走天数刚好为n个月完整天数相加时（31+28+31...））￿??
            prtc->date.month = i + 1;         // 月份不加1，日期溢出（如：出现32号）
            prtc->date.date = count_temp + 1; // 应不作处理，CounterTemp = month[i] = 31时，会继续循环，月份加一￿??
            break;                            // 日期变为1，刚好符合实际日￿??
        }

        count_temp -= month[i];
    }

    get_weekday(prtc);
}
#endif

#if defined(KALMAN)
#define EPS 1e-8
/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp, float input)
{
    /*预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差*/
    kfp->Now_Covariance = kfp->Last_Covariance + kfp->Q;
    /*卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）*/
    kfp->Kg = kfp->Now_Covariance / (kfp->Now_Covariance + kfp->R);
    /*更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）*/
    kfp->Output = kfp->Output + kfp->Kg * (input - kfp->Output); // 因为这一次的预测值就是上一次的输出值
    /*更新协方差方程: 本次的系统协方差付给 kfp->Last_Covariance 为下一次运算准备。*/
    kfp->Last_Covariance = (1 - kfp->Kg) * kfp->Now_Covariance;
    /*当kfp->Output不等于0时，负方向迭代导致发散到无穷小*/
    if (kfp->Output - 0.0 < EPS)
    {
        kfp->Kg = 0;
        kfp->Output = 0;
    }
    return kfp->Output;
}
#else
/**
 *滑动滤波器
 *@param SideParm *side 滑动结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float sidefilter(SideParm *side, float input)
{
    // 第一次滤波
    if (side->First_Flag)
    {

        for (int i = 0; i < sizeof(side->SideBuff) / sizeof(float); i++)
        {
            side->SideBuff[i] = input;
        }

        side->First_Flag = false;
        side->Head = &side->SideBuff[0];
        side->Sum = input * (sizeof(side->SideBuff) / sizeof(float));
    }
    else
    {
        side->Sum = side->Sum - *side->Head + input;
        *side->Head = input;

        if (++side->Head > &side->SideBuff[sizeof(side->SideBuff) / sizeof(float) - 1])
        {
            side->Head = &side->SideBuff[0];
        }

        input = side->Sum / (1 << FILTER_SHIFT);
    }

    return input;
}
#endif

void dis_hex_data(uint8_t *dat, uint16_t width, uint32_t len)
{
#define HEX_HEAD_SIZE sizeof("\r\n[%04x]: ") - 1
#define HEX_VAL_SIZE sizeof("     ") - 1
#define HEX_TOTAL_SIZE (2 + len + HEX_HEAD_SIZE + width * HEX_VAL_SIZE)
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
    uint16_t i, j, k = 0;

    if (NULL == dat || !width || !len)
        return;

    char *buf = (char *)pvPortMalloc(HEX_TOTAL_SIZE);
    if (NULL == buf)
        goto __exit;

    memset(buf, 0, HEX_TOTAL_SIZE);

    for (i = 0; i < len; i += width)
    {
        // shellPrint(Shell_Object, "\r\n[%04x]: ", i);
        sprintf(buf, "\r\n[%04x]: ", i);
        k = sizeof("\r\n[%04x]: ") - 1;
        for (j = 0; j < width; ++j)
        {
            if (i + j < len)
            {
                // shellPrint(Shell_Object, "%#02x ", dat[i + j]);
                sprintf(&buf[k], "%#02x ", dat[i + j]);
            }
            else
            {
                // shellPrint(Shell_Object, "   ");

                memcpy(&buf[k], "     ", 5);
            }
            k += sizeof("     ") - 1;
        }
        // shellPrint(Shell_Object, "\t\t");

        memcpy(&buf[k], "\t\t", 2);
        k += 2U;
        for (j = 0; (i + j < len) && j < width; ++j)
        {
            // shellPrint(Shell_Object, "%c",
            //            __is_print(dat[i + j]) ? dat[i + j] : '.');
            sprintf(&buf[k], "%c", __is_print(dat[i + j]) ? dat[i + j] : '.');
            k += 1;
        }
        // shellWriteString(Shell_Object, buf);
        (Shell_Object)->write(buf, k);
        k = 0;
    }
    if (len)
        shellWriteString(Shell_Object, "\r\n\r\n");
__exit:
    vPortFree(buf);
}

/**
 * @brief	查看板卡信息
 * @details
 * @param	None
 * @retval	None
 */
void see_card_info(void)
{
    Shell *sh = shellGetCurrent();

    shellPrint(sh, "\r\nIRQ_Table Information:\r\nSlaveId\t\tPriority\t\tType\t\tNumber\r\n");
    for (uint8_t i = 0; i < CARD_NUM_MAX; i++)
    {
        shellPrint(sh, "0x%x\t\t0x%x\t\t\t0x%x\t\t%d\r\n",
                   IRQ_Table.pIRQ[i].SlaveId, IRQ_Table.pIRQ[i].Priority,
                   IRQ_Table.pIRQ[i].TypeCoding, IRQ_Table.pIRQ[i].Number);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 see_card_info, see_card_info, Viewing Board Information.);

/**
 * @brief	查看板卡信息
 * @details
 * @param	None
 * @retval	None
 */
void see_param_info(void)
{
    Shell *sh = shellGetCurrent();

    float *pdata = (float *)&Save_Flash.Param;
    uint32_t actual_size = offsetof(Save_Param, Error_Code);

    shellPrint(sh, "\r\np\t\t*p\r\n");
    for (float *p = pdata; p < pdata + actual_size / sizeof(float); p++)
    {
        shellPrint(sh, "%p\t%.3f\r\n", p, *p);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 see_param_info, see_param_info, View parameter information.);

/**
 * @brief	设置各模块的动态debug开关
 * @details
 * @param	None
 * @retval	None
 */
int set_dbug_switch(int argc, char *argv[])
{
    Shell *sh = shellGetCurrent();
    if (argc < 3)
    {
        shellPrint(sh, "@error:too few parameter,please input'<set_dbug [bitx|val]>.'\r\n");
        return -1;
    }
    if (argc > 3)
    {
        shellPrint(sh, "@error:too many parameter,please input'<set_dbug [bitx|val]>.'\r\n");
        return -1;
    }
    uint8_t bit = (uint8_t)atoi(argv[1]);
    uint8_t val = (uint8_t)atoi(argv[2]);

    if (bit > 31)
    {
        shellPrint(sh, "@error: Illegal flag bit(must < 32).'\r\n");
        return -1;
    }
    if (val > 1)
    {
        shellPrint(sh, "@error: Illegal flag bit(must 0 | 1).'\r\n");
        return -1;
    }

    if (val)
        g_Debug_Bits |= 1 << bit;
    else
        g_Debug_Bits &= ~(1 << bit);

    shellPrint(sh, "dbug %#x.'\r\n", g_Debug_Bits);

    return 0;
}
SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
    set_debug, set_dbug_switch, set dbug switch.);

/**
 * @brief	重启系统
 * @details
 * @param	None
 * @retval	None
 */
void reboot(void)
{
    Shell *sh = shellGetCurrent();
    shellPrint(sh, "start reboot, wait...\r\n");

    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 复位
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 reboot, reboot, Restart the system.);

/**
 * @brief	查看RTOS目前可用内存
 * @details
 * @param	None
 * @retval	None
 */
void free_mem(void)
{
    Shell *sh = shellGetCurrent();
    shellPrint(sh, "\r\n\r\nheap(byte)\tmini_heap\r\n%u\t\t%u\r\n",
               xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 free, free_mem, View system remaining memory.);

/**
 * @brief	查看RTOS线程情况
 * @details
 * @param	None
 * @retval	None
 */
void list_thread(void)
{
    Shell *sh = shellGetCurrent();

#if ((configUSE_TRACE_FACILITY == 1) &&            \
     (configUSE_STATS_FORMATTING_FUNCTIONS > 0) && \
     (configSUPPORT_DYNAMIC_ALLOCATION == 1))
    TaskStatus_t *pbuf = pvPortMalloc(uxTaskGetNumberOfTasks() * sizeof(TaskStatus_t) * 10U);
    if (pbuf)
    {
        vTaskList((char *)pbuf);
        shellPrint(sh, "Name\t\tState\tPrio\tRStack\tId\r\n");
        shellPrint(sh, "%s\r\n", pbuf);
        vTaskGetRunTimeStats((char *)pbuf);
        shellPrint(sh, "Name\t\tRunCount\tCpuUsage\r\n");
        shellPrint(sh, "%s\r\n", pbuf);
    }
    vPortFree(pbuf);
#endif
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                 list_thread, list_thread, Viewing System Threads.);
