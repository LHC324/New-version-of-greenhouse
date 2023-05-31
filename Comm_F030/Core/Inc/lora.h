#ifndef _LORA_H_
#define _LORA_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "tool.h"
#if defined(USING_RTTHREAD)
#include "rtthread.h"
#else
#include "cmsis_os.h"
#include "shell_port.h"
#endif

#define LORA_USING_DEBUG 0
#define LORA_SHELL &shell
#define LORA_DEBUG(fmt, ...) shellPrint(LORA_SHELL, fmt, ##__VA_ARGS__)
#define lora_malloc pvPortMalloc
#define lora_free vPortFree
#define lora_delay osDelay
#define USING_FREERTOS
/*lora使用中继模式*/
#define USING_REPEATER_MODE
/*1级中继地址*/
#define USING_REPEATER_1_ADDR (0x01)
/*支持的最大从机*/
#define SLAVE_MAX_NUMBER 16U
#define LORA_TX_BUF_SIZE 64U
#define LORA_RX_BUF_SIZE 64U
/*不同类型从机的数量*/
#define LORA_DI_SLAVE_MAX_NUM 4U
#define LORA_D0_SLAVE_MAX_NUM 8U
#define LORA_AI_SLAVE_MAX_NUM 4U
#define LORA_AO_SLAVE_MAX_NUM 0U
/*A型从机数字输入路数*/
#define DIGITAL_INPUT_NUMBERS 8U
#define DIGITAL_INPUT_OFFSET 1U
#define DIGITAL_OUTPUT_OFFSET 5U

#define ANALOG_INPUT_NUMBERS 8U
/*Lora模块调度时间*/
#define LORA_SCHEDULE_TIMES 200U // cotex m0内核速度低于cotex m1，此时间不宜太小，否则接收中断反应不了
/*Lora模块无效ID号*/
#define LORA_NULL_ID 0xFF
/*Lora从机离线/在线情况统计，存储时偏移量*/
#define LORA_STATE_OFFSET (LORA_DI_SLAVE_MAX_NUM * 8U) // 32U

/*定义Master发送缓冲区字节数*/
#define PF_TX_SIZE 64U
#define LEVENTS (sizeof(L101_Map) / sizeof(L101_HandleTypeDef))
/*累计3次超时或者错误后，改变上报的时间*/
#define SUSPEND_TIMES 3U
/*30s增加一个离线设备扫描*/
#define TOTAL_SUM 5U
/*2轮调度均离线：则认为从机掉线，主动清除状态*/
#define SCHEDULING_COUNTS 2U
    /*Enter键值*/
    // #define ENTER_CODE 0x2F
    typedef enum
    {
        Lora_Nomal_Recive = 0,
        Lora_Param_Cfg,
        Lora_Unknown_Mode,
    } Lora_Work_Mode;

    typedef enum
    {
        Lora_Di_Slave,
        Lora_Do_Slave,
        Lora_Ai_Slave,
        Lora_Ao_Slave,
        Lora_Unknown_Slave,
    } Lora_Slave_Type;

    typedef enum
    {
        L_None,
        L_Wait,
        L_OK,
        L_Error,
        L_TimeOut,
    } Lora_State;

    typedef struct
    {
        struct
        {
            union
            {
                uint16_t Val;
                struct
                {
                    uint8_t L;
                    uint8_t H;
                } V;
            } Device_Addr;
            uint8_t Channel;
        } Frame_Head;

        struct
        { /*当前状态*/
            Lora_State State;
            /*超时时间*/
            uint8_t OverTimes;
            /*错误计数器*/
            uint8_t Counter;
            uint8_t Schedule_counts; /*调度次数*/
        } Check;

    } Lora_Map;

    typedef struct
    {
        List_t *pList;
        ListItem_t *pIter;
    } Listx_t;

    typedef struct Lora_HandleTypeDef *pLoraHandle;
    typedef struct Lora_HandleTypeDef Lora_Handle;
    struct Lora_HandleTypeDef
    {
        /*用于解决lora模块通过shell参数时，模式区分*/
        Lora_Work_Mode Mode;
        struct
        {
            Lora_State State;        /*当前状态*/
            uint8_t OverTimes;       /*超时时间*/
            uint8_t Counter;         /*错误计数器*/
            uint8_t Schedule_counts; /*调度次数*/
        } Check;
        /*建立lora模块各节点间调度数据结构*/
        struct
        {
            List_t *Ready, *Block;
            uint8_t Event_Id;
            uint8_t Period; /*阻塞设备调度周期*/
            bool First_Flag;
        } Schedule;
        void (*Set_Lora_Factory)(void);
        bool (*Get_Lora_Status)(pLoraHandle);
        // bool (*Lora_Handle)(pLoraHandle);
        void (*Lora_Recive_Poll)(pLoraHandle);
        void (*Lora_Transmit_Poll)(pLoraHandle);
        // void (*Lora_TI_Recive)(pLoraHandle, DMA_HandleTypeDef *);
        // bool (*Lora_MakeFrame)(pLoraHandle, Lora_Map *);
        void (*Lora_Send)(pLoraHandle, uint8_t *, uint16_t);
        bool (*Lora_Check_InputCoilState)(uint8_t *, uint8_t *, uint8_t);
        void (*Lora_CallBack)(pLoraHandle, void *);
        // UART_HandleTypeDef *huart;
        UartHandle Uart;
        Gpiox_info Cs;
        /*预留外部接口*/
        void *pHandle;
    } __attribute__((aligned(4)));

    extern pLoraHandle Lora_Object;
    extern void MX_Lora_Init(void);

#define Clear_LoraBuffer(__ptr, name)                                     \
    do                                                                    \
    {                                                                     \
        (__ptr)                        ? memset((__ptr)->name.rx.pbuf, 0, \
                                                (__ptr)->name.rx.count),  \
            (__ptr)->name.rx.count = 0 : false;                           \
    } while (0)

#define Check_FirstFlag(__ptr)                                                                          \
    do                                                                                                  \
    {                                                                                                   \
        if (!(__ptr)->Schedule.First_Flag)                                                              \
        {                                                                                               \
            (__ptr)->Schedule.First_Flag =                                                              \
                (++(__ptr)->Schedule.Event_Id >= (__ptr)->MapSize) ? (__ptr)->Schedule.Event_Id = 0xFF, \
            true                                                                                        \
                : false;                                                                                \
        }                                                                                               \
    } while (0)

/*获取主机号*/
#define Get_LoraId(__obj) ((__obj)->Uart.rx.pbuf[0U])
    // #define Lora_ReciveHandle(__obj, __dma) ((__obj)->Lora_TI_Recive((__obj), (__dma)))

#ifdef __cplusplus
}
#endif

#endif /* _L102_H_*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
