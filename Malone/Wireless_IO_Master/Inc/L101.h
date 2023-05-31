#ifndef _L101_H_
#define _L101_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

/*定义Master发送缓冲区字节数*/
#define PF_TX_SIZE 64U
#define LEVENTS (sizeof(L101_Map) / sizeof(L101_HandleTypeDef))
/*累计三次超时或者错误后，改变上报的时间*/
#define SUSPEND_TIMES 3U
/*30s增加一个离线设备扫描*/
#define TOTAL_SUM 5U
    /*Enter键值*/
    // #define ENTER_CODE 0x2F

    typedef struct
    {
        union
        {
            uint16_t Val;
            struct
            {
                uint8_t H;
                uint8_t L;
            } V;
        } Addr;
        // uint8_t Sch;
        // uint8_t *pData;
    } Frame_Head __attribute__((aligned(2)));

    typedef enum
    {
        L_None,
        L_Wait,
        L_OK,
        L_Error,
        L_TimeOut,
    } L101_State;

    /*定义L101事件处理结构*/
    typedef struct L101
    { /*从站设备地址*/
        uint16_t Sdevice_Addr;
        /*从站信道*/
        uint8_t Schannel;
        /*从站号*/
        uint8_t Slave_Id;
        /*数字量开关信号存储地址*/
        uint16_t Digital_Addr;
        /*模拟量信号地址*/
        uint16_t Analog_Addr;
        uint16_t Crc16;
        /*检查各从站是否响应*/
        struct
        { /*当前状态*/
            L101_State State;
            /*超时时间*/
            uint32_t Times;
            /*错误计数器*/
            uint32_t Counter;
        } Check;
        /*对应回调函数*/
        uint8_t (*func)(struct L101 *param);
    } L101_HandleTypeDef __attribute__((aligned(4)));

    extern void Slist_Init(void);
    extern bool inline Get_L101_Status(void);
    extern void Set_L101_FactoryMode(void);
    extern void Shell_Mode(void);
    extern void Master_Poll(void);
#ifdef __cplusplus
}
#endif

#endif /* _L101_H_*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
