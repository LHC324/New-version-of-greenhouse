#ifndef __TOOL_H__
#define __TOOL_H__

#ifdef __cplusplus
extern "C"
{
#endif
    // #include "main.h"

#define __INFORMATION()                            \
    "Line: %d,  Date: %s, Time: %s, Name: %s\r\n", \
        __LINE__, __DATE__, __TIME__, __FILE__
/*Configuration Wizard：https://blog.csdn.net/qq_15647227/article/details/89297207*/
// <<< Use Configuration Wizard in Context Menu >>>

// <s>Define Tool Config File Version
//  <i>version
#define TOOL_CONFIG_VERSION "1.5.0"

// <o>Selsect the current running environment of the mcu
//  <i>Default: 2
//  <0=> Bare pager
//  <1=> Freertos
//  <2=> rt_thread
/*使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define TOOL_USING_RTOS 1

// <o>Enable stm32HAL library
//  <i>Default: 1
//  <0=> DisEable
//  <1=> Eable
/*使用STM32HAL库*/
#define TOOL_USING_STM32HAL 1

// <o>Enables the exchange of any type of data tool
//  <i>Default: 1
//  <0=> DisEable
//  <1=> Eable
/*交换任意类型数据*/
#define TOOL_USING_SWAP_ANY 1

// <o>Selsect filtering algorithm
//  <i>Default: 1
//  <0=> Sliding filtering
//  <1=> Kalman filtering
/*使用卡尔曼滤波*/
#define TOOL_USING_KALMAN 1

// <o>Enables crc16 algorithm
//  <i>Default: 1
//  <0=> DisEable
//  <1=> Eable
/*使用CRC16*/
#define TOOL_USING_CRC16 1

// <o>Enable big and small end exchange algorithm
//  <i>Default: 1
//  <0=> DisEable
//  <1=> Eable
/*使用大小端字节交换*/
#define TOOOL_USING_ENDIAN 1

// <o>Enable position pid algorithm
//  <i>Default: 0
//  <0=> DisEable
//  <1=> Eable
/*位置式pid*/
#define TOOOL_USING_SITE_PID 0

// <o>Enable incremental pid algorithm
//  <i>Default: 0
//  <0=> DisEable
//  <1=> Eable
/*增量式pid*/
#define TOOOL_USING_INCREMENT_PID 0

// <o>Enable Linux timestamp tool
//  <i>Default: 0
//  <0=> DisEable
//  <1=> Eable
/*Linux时间戳*/
#define TOOOL_USING_LINUX_STAMP 0

#if (TOOL_USING_RTOS == 1)
#include "cmsis_os.h"
#define release_semaphore osSemaphoreRelease
#elif (TOOL_USING_RTOS == 2)
#include <rtthread.h>
#define release_semaphore rt_sem_release
#endif
#if (TOOL_USING_STM32HAL)
// #define TOOL_STM32_HAL_H "main.h"
#include "main.h"
#endif
// <<< end of configuration section >>>

/*连接两个字符串S1、S2*/
#define Connect_Str(S1, S2) (S1##S2)
/*按位操作任意类型数据*/
#define __SET_FLAG(__OBJECT, __BIT) (((__OBJECT) |= 1U << (__BIT)))
#define __RESET_FLAG(__OBJECT, __BIT) (((__OBJECT) &= ~(1U << (__BIT))))
#define __GET_FLAG(__OBJECT, __BIT) (((__OBJECT) & (1U << (__BIT))))
#if (TOOL_USING_SWAP_ANY)
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

#if (TOOOL_USING_SITE_PID)
    /*位置式PID*/
    typedef struct
    {
        float kp;      // 比列系数
        float ki;      // 积分系数
        float kd;      // 微分系数
        float last_ek; // 上一次误差
        float sum_ek;  // 累计误差
    } site_pid_t;

    extern void init_site_pid(site_pid_t *pid,
                              float kp,
                              float ki,
                              float kd);
    extern float get_site_pid_out(site_pid_t *pid, float cur_val, float tar_val);
#endif

#if (TOOOL_USING_INCREMENT_PID)
    /*增量式PID*/
    typedef struct
    {
        float kp;           // 比列系数
        float ki;           // 积分系数
        float kd;           // 微分系数
        float last_ek;      // 上一次误差
        float last_last_ek; // 上一次的上一次误差
    } incremental_pid_t;

    extern void init_increment_pid(incremental_pid_t *pid,
                                   float kp,
                                   float ki,
                                   float kd);
    extern float get_incremental_pid_out(incremental_pid_t *pid,
                                         float cur_val,
                                         float tar_val);
#endif

#if (TOOOL_USING_LINUX_STAMP == 1)
    typedef struct
    {
        struct
        {
            unsigned char year;
            unsigned char month;
            unsigned char date;
            unsigned char weelday;
        } date;
        struct
        {
            unsigned char hours;
            unsigned char minutes;
            unsigned char seconds;
        } time;
    } rtc_t;
    extern unsigned int std_time_to_linux_stamp(rtc_t *prtc);
    extern void get_weekday(rtc_t *prtc);
    extern void linux_stamp_to_std_time(rtc_t *prtc, unsigned int cur_stamp, int time_zone);
#endif
    typedef struct
    {
        void *pGPIOx;
        unsigned short Gpio_Pin;
        // unsigned char PinState;
    } Gpiox_info;

    typedef enum
    {
        co_string,
        co_bool,
        co_uint8,
        co_int8,
        co_uint16,
        co_int16,
        co_uint32,
        co_int32,
        co_float,
        co_long,
        co_ulong,
        co_type_max,
    } comm_data_type;
    typedef struct
    {
        // unsigned short index;
        void *val;
        comm_data_type type;
    } comm_val_t;

    extern const char *text_name[co_type_max + 1U];

#define BY_DATA_TYPE_GET_SIZE(_type)                                                                                                \
    (_type) > co_string && (_type)<co_uint16 ? 1U : (_type)> co_int8 && (_type)<co_uint32 ? 2U : (_type)> co_type_max ? co_type_max \
                                                                                                                      : 4U

#if (TOOL_USING_KALMAN)
/*以下为卡尔曼滤波参数*/
#define LASTP 0.500F   // 上次估算协方差
#define COVAR_Q 0.005F // 过程噪声协方差
#define COVAR_R 0.067F // 测噪声协方差

    typedef struct
    {
        float Last_Covariance; // 上次估算协方差 初始化值为0.02
        float Now_Covariance;  // 当前估算协方差 初始化值为0
        float Output;          // 卡尔曼滤波器输出 初始化值为0
        float Kg;              // 卡尔曼增益 初始化值为0
        float Q;               // 过程噪声协方差 初始化值为0.001
        float R;               // 观测噪声协方差 初始化值为0.543
    } KFP;

    extern float kalmanFilter(KFP *kfp, float input);
#else
/*左移次数*/
#define FILTER_SHIFT 4U

typedef struct
{
    bool First_Flag;
    float SideBuff[1 << FILTER_SHIFT];
    float *Head;
    float Sum;
} SideParm;
extern float sidefilter(SideParm *side, float input);
#endif

    typedef void (*_func)(void *, unsigned char, unsigned short);
    typedef struct
    {
        uint16_t addr;
        float upper;
        float lower;
        _func event;
    } Event_Map;
#define _dwin_func _func

    typedef enum
    {
        uart_using_it,
        uart_using_dma,
        uart_none_mode = 0xFF,
    } uart_wmode;
    typedef struct
    {
        uart_wmode wmode;
        unsigned char *pbuf;
        unsigned int size, count;
    } Uart_Data_HandleTypeDef __attribute__((aligned(4)));

    typedef struct Uart_HandleTypeDef *pUartHandle;
    typedef struct Uart_HandleTypeDef UartHandle;
    struct Uart_HandleTypeDef
    {
        void *huart, *phdma;
        Uart_Data_HandleTypeDef rx, tx;
#if (!TOOL_USING_RTOS)
        bool recive_finish_flag;
#else
    void *semaphore;
#endif
    } __attribute__((aligned(4)));

#if (TOOL_USING_STM32HAL)
#define uartx_irq_recive(obj, name) (obj->name##_Recive(&(obj->Uart)))
    void uartx_recive_handle(pUartHandle pu);
#endif
#if (TOOL_USING_CRC16)
    unsigned short get_crc16(unsigned char *ptr, unsigned short length, unsigned short init_dat);
#endif
#if (TOOOL_USING_ENDIAN)
    void endian_swap(unsigned char *pdata, unsigned char start, unsigned char length);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TOOL_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
