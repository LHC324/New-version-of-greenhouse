#ifndef __TOOL_H__
#define __TOOL_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#if defined(USING_FREERTOS)
#include "cmsis_os.h"
#endif

#define __INFORMATION()                            \
    "Line: %d,  Date: %s, Time: %s, Name: %s\r\n", \
        __LINE__, __DATE__, __TIME__, __FILE__

#define DEBUG_APP_BITT (1 << 0)
#define DEBUG_IRQ_BITT (1 << 1)
#define DEBUG_MODBUS_BITT (1 << 2)
#define DEBUG_DWIN_BITT (1 << 3)
#define DEBUG_LTE_BITT (1 << 4)
#define DEBUG_WIFI_BITT (1 << 5)
#define DEBUG_RS485_BITT (1 << 6)

//使用Linux时间戳
#define TOOOL_USING_LINUX_STAMP 1

// /*使用就绪列表*/
// #define USING_FREERTOS_LIST 0
/*交换任意类型数据*/
#define USING_SWAP_ANY 1

    /*使用卡尔曼滤波*/
    // #define KALMAN

#if defined(KALMAN)
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

    typedef struct _Range
    {
        int start, end;
    } Range __attribute__((aligned(4)));

    typedef struct DMA_HandleTypeDef *pDmaHandle;
    typedef struct DMA_HandleTypeDef DmaHandle;
    struct DMA_HandleTypeDef
    {
        UART_HandleTypeDef *huart;
        DMA_HandleTypeDef *phdma;
        uint8_t *pRbuf;
        uint8_t RxSize;
        uint32_t *pRxCount;
#if !defined(USING_FREERTOS)
        bool *pRecive_FinishFlag;
#else
    osSemaphoreId Semaphore;
#endif
    } __attribute__((aligned(4)));

/*硬件支持的板卡数*/
// #define CARD_NUM_MAX 0x10
/*板卡类型数*/
#define CARD_TYPE_MAX 0x08
/*每块板卡的信号数*/
#define CARD_SIGNAL_MAX 0x08
/*板卡类型的优先级上限*/
#define PRIORITY_MIN 0x00
/*板卡类型的优先级上限*/
#define PRIORITY_MAX 0x50
/*无效中断源位置*/
#define INACTIVE_SITE 0x00
/*通信类板卡地址偏移量:32bit数据+32bit从机状态*/
#define CARD_COMM_OFFSET_MAX 0x40

/*是否使用用户定时器*/
#define USING_USERTIMER0 0
#define USING_USERTIMER1 0
#define START_SIGNAL_MAX 10U
#define USER_COIL_OFFSET 4U
#define BX_SIZE 7U
#define TEST_BX_SIZE 4
#define VX_SIZE 32U             // 5U
#define AO_SIZE (8U * 4U)       /*模拟量输出占用的字节数*/
#define DIGITAL_OUTPUTOFFSET 4U // 有线阀门偏移量
#define T_5S 5U
#define CURRENT_UPPER 16.0F
#define CURRENT_LOWER 4.0F
#define PI acosf(-1.0F)
#define Get_Target(__current, __upper, __lower) \
    (((__current)-CURRENT_LOWER) / CURRENT_UPPER * ((__upper) - (__lower)) + (__lower))
#define Get_Ptank_Level(__H, __HI, __CR, __CL)                                                                      \
    ((__CL) * (PI * powf((__CR), 2.0F) / 2.0F - ((__CR) - (__H)) * sqrtf(2.0F * (__CR) * (__H)-powf((__H), 2.0F)) - \
               powf((__CR), 2.0F) * asinf(((__CR) - (__H)) / (__CR))) +                                             \
     (PI * (__HI) / 3.0F * (__CR) * (3.0F * powf((__CR), 2.0F) * (__H)-powf((__CR), 3.0F) + powf((__CR) - (__H), 3.0F))))
#define Get_Ptank_Level_1(__H, __CR, __CL)                                           \
    ((1.0F + 2.0F * (__CR) / (3.0F * (__CL))) * ((__CL)*powf((__CR), 2.0F) *         \
                                                 (asinf(((__H) - (__CR)) / (__CR)) + \
                                                  ((__H) - (__CR)) / powf((__CR), 2.0F) * sqrtf(powf((__CR), 2.0F) - powf(((__H) - (__CR)), 2.0F)) + PI / 2.0F)))
#define Set_SoftTimer_Flag(__obj, __val) \
    ((__obj)->flag = (__val))
#define Set_SoftTimer_Count(__obj, __val) \
    (__obj)->flag ? false : ((__obj)->counts = (__val), (__obj)->flag = true)
#define Reset_SoftTimer(__obj, __val) \
    ((__obj)->counts = (__val), (__obj)->flag = false)
#define SoftTimer_IsTrue(__obj) \
    do                          \
    {                           \
        if ((__obj)->counts)    \
        {                       \
            (__obj)->counts--;  \
            goto __no_action;   \
        }                       \
    } while (0);
#define Clear_Counter(__obj) ((__obj)->counts = 0U)
#define Open_Qx(__x) ((__x) < VX_SIZE ? wbit[__x] = true : false)
#define Close_Qx(__x) ((__x) < VX_SIZE ? wbit[__x] = false : false)
#define TARGET_BOARD_NUM ((VX_SIZE + (CARD_SIGNAL_MAX - 1U)) / CARD_SIGNAL_MAX)
#define __Get_TargetBoardNum(bytes, card_size) \
    (card_size ? (bytes + (card_size - 1U)) / card_size : 0U)

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
        // IRQ_Code *pTIRQ;
        uint8_t *p_id;   /*指向目标板卡id存储对象*/
        uint8_t id_size; /*id存储的数量*/
        uint8_t count;   /*实际找到到的目标id数*/
    } R_TargetTypeDef;

    typedef struct
    {
        Slave_IRQTableTypeDef *sp;
        R_TargetTypeDef *pr;
        Card_Tyte ctype;
        uint8_t nums, *pwbit; /*用户程序实际使用目标id板卡的数目；用户数据源*/
        uint16_t wbit_size;   /*用户数据源大小*/
    } Report_TypeDef;

    typedef struct
    {
        void *pGPIOx;
        unsigned short Gpio_Pin;
        // unsigned char PinState;
    } Gpiox_info;

    extern IRQ_Code IRQ[CARD_NUM_MAX];
    extern Slave_IRQTableTypeDef IRQ_Table;
#define QUICK_SORT_TYPE IRQ_Request
    extern void Quick_Sort(QUICK_SORT_TYPE *pData, int len);
    extern void Add_ListItem(List_t *list, TickType_t data);
    extern ListItem_t *Find_ListItem(List_t *list, TickType_t data);
    extern UBaseType_t Remove_ListItem(List_t *list, TickType_t data);
    extern ListItem_t *Get_OneListItem(List_t *list, ListItem_t **p);
    extern IRQ_Code *Find_TargetSlave_AdoptId(Slave_IRQTableTypeDef *irq, uint8_t target_id);
    extern IRQ_Code *Find_TargetSlave_AdoptType(Slave_IRQTableTypeDef *irq, IRQ_Code *p_current);
    extern bool Save_TargetSlave_Id(Slave_IRQTableTypeDef *irq,
                                    Card_Tyte type,
                                    R_TargetTypeDef *psave,
                                    uint8_t nums);
    extern void IRQ_Coding(Slave_IRQTableTypeDef *irq, uint8_t code);
    extern uint16_t Get_Crc16(uint8_t *ptr, uint16_t length, uint16_t init_dat);
    extern void Endian_Swap(uint8_t *pData, uint8_t start, uint8_t length);
    extern void DMA_Recive(pDmaHandle pd);
#define DMA_ReciveHandle(__obj) (DMA_Recive(&(__obj)->Uart))
#define DMA_ReciveHandl1(__obj) (DMA_Recive(&(__obj)))

    extern void dis_hex_data(uint8_t *dat, uint16_t width, uint32_t len);
    extern unsigned int g_Debug_Bits;
#ifdef __cplusplus
}
#endif

#endif /* __TOOL_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
