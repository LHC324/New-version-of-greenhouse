#ifndef _IO_SIGNAL_H_
#define _IO_SIGNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

/*定义外部数字量输入路数*/
#define EXTERN_DIGITALIN_MAX 8U
/*定义外部模拟量输入路数*/
#define EXTERN_ANALOGIN_MAX 8U
/*定义外部数字量输出路数*/
#define EXTERN_DIGITALOUT_MAX 8U
/*定义外部模拟量输入路数*/
#define EXTERN_ANALOGOUT_MAX 8U
/*输入数字信号量在内存中初始地址*/
#define INPUT_DIGITAL_START_ADDR 0x00
/*输出数字信号量在内存中初始地址*/
#define OUT_DIGITAL_START_ADDR 0x00
/*输入模拟信号量在内存中初始地址*/
#define INPUT_ANALOG_START_ADDR 0x00
/*输出模拟信号量在内存中初始地址*/
#define OUT_ANALOG_START_ADDR 0x00
/*检测输入引脚延时*/
#define CHECK_DELAY_5MS 5U

    typedef struct
    {
        GPIO_TypeDef *pGPIOx;
        uint16_t GPIO_Pinx;
    } Gpiox_t;

    typedef struct
    {
        Gpiox_t gpio;
        uint32_t count;
    } di_input_group_t;

    typedef struct
    {
        di_input_group_t *group;
        uint16_t group_size;
        uint16_t site, bits;
        void *pQueue;
    } di_input_t __attribute__((aligned(4)));

    extern di_input_t dix;
    extern void di_timeout_check(di_input_t *pi);

    extern void Read_Digital_Io(void *arg);
    extern void Read_Analog_Io(void *arg);
    extern void Write_Digital_IO(void *arg);
    extern void Write_Analog_IO(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __IO_SIGNAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
