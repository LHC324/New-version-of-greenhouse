#include "io_signal.h"
#include "main.h"
#include "tool.h"
#include "Modbus.h"
#if defined(USING_INPUT_REGISTER) || defined(USING_HOLD_REGISTER)
#include "adc.h"
#include "Mcp4822.h"
#endif

/**
 * @brief	外部数字量输入处理
 * @details	STM32F030F4共在io口扩展了8路数字输入
 * @param	None
 * @retval	None
 */
#if defined(USING_INPUT_COIL)
void Read_Digital_Io(void *arg)
{
    pModbusHandle pd = Modbus_Object;
    di_input_t *pi = (di_input_t *)arg;
    uint8_t wbits[EXTERN_DIGITALIN_MAX];
    /*备份记录区*/
    static uint8_t history_wbits = 0x00;

    for (uint8_t i = 0; pd && pi && i < EXTERN_DIGITALIN_MAX; i++)
    {
        wbits[i] = (pi->bits >> i) & 0x01;
        /*只要有一个状态和上一次不同，则发出中断请求*/
        if ((uint8_t)((history_wbits >> i) & 0x01) ^ wbits[i])
        {
            *(bool *)pd->Slave.pHandle = false;
#if defined(USING_DEBUG)
            Debug("\r\nInput coil[0x%d] = 0x%x status change!\r\n", i, wbits[i]);
#endif
            /*当前状态为下一次历史记录*/
            if (wbits[i])
                history_wbits |= (uint8_t)(1U << i);
            else
                history_wbits &= ~(uint8_t)(1U << i);
        }
    }
    pi->bits = 0; // 清空本次结果
    if (pd)
    {
        pd->Slave.Reg_Type = InputCoil;
        pd->Slave.Operate = Write;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, INPUT_DIGITAL_START_ADDR, wbits, EXTERN_DIGITALIN_MAX))
        {
#if defined(USING_DEBUG)
            Debug("Input coil write failed!\r\n");
#endif
        }
    }
}

di_input_group_t di_group[EXTERN_DIGITALIN_MAX] = {
    {
        {DI0_GPIO_Port, DI0_Pin},
        0,
    },
    {
        {DI1_GPIO_Port, DI1_Pin},
        0,
    },
    {
        {DI2_GPIO_Port, DI2_Pin},
        0,
    },
    {
        {DI3_GPIO_Port, DI3_Pin},
        0,
    },
    {
        {DI4_GPIO_Port, DI4_Pin},
        0,
    },
    {
        {DI5_GPIO_Port, DI5_Pin},
        0,
    },
    {
        {DI6_GPIO_Port, DI6_Pin},
        0,
    },
    {
        {DI7_GPIO_Port, DI7_Pin},
        0,
    },
};

di_input_t dix = {
    .group = di_group,
    .group_size = sizeof(di_group) / sizeof(di_input_group_t),
    .pQueue = NULL,
};

/**
 * @brief  Timer timeout detection.
 * @param  None
 * @retval None
 */
void di_timeout_check(di_input_t *pi)
{
    uint8_t offset = 0;

    for (di_input_group_t *p = pi->group;
         pi && pi->group && p < pi->group + pi->group_size; ++p, ++offset)
    {
        if (p->count)
        {
            p->count--;
        }
        else
        {
            p->count = CHECK_DELAY_5MS;

            /*读取外部数字引脚状态:翻转光耦的输入信号*/
            if (p->gpio.pGPIOx &&
                !HAL_GPIO_ReadPin(p->gpio.pGPIOx, p->gpio.GPIO_Pinx))
            {
                pi->bits |= 1U << offset;
            }
        }
    }
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    di_input_t *pi = &dix;

    for (di_input_group_t *p = pi->group;
         pi->group && p < pi->group + pi->group_size; ++p)
    {
        if (p->gpio.GPIO_Pinx == GPIO_Pin)
        {
            p->count = CHECK_DELAY_5MS;
        }
    }
}
#endif

/**
 * @brief	数字量输出
 * @details	STM32F030F4共在io口扩展了8路数字输出
 * @param	None
 * @retval	None
 */
#if defined(USING_COIL)
void Write_Digital_IO(void *arg)
{
    /*引脚默认指向输入第一路*/
    GPIO_TypeDef *pGPIOx = NULL;
    uint16_t GPIO_Pinx = 0x00;
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[EXTERN_DIGITALOUT_MAX];

    if (pd && pGPIOx)
    {
        pd->Slave.Reg_Type = Coil;
        pd->Slave.Operate = Read;
        /*读取对应寄存器*/
        if (!pd->Mod_Operatex(pd, OUT_DIGITAL_START_ADDR, rbits, EXTERN_DIGITALOUT_MAX))
        {
#if defined(USING_DEBUG)
            Debug("Coil reading failed!\r\n");
            return;
#endif
        }

        for (uint8_t i = 0; i < EXTERN_DIGITALOUT_MAX; i++)
            HAL_GPIO_WritePin(pGPIOx, GPIO_Pinx, (GPIO_PinState)rbits[i]);
    }
}

#endif

/**
 * @brief	外部模拟量输入处理
 * @details	STM32F030F4共在io口扩展了8路模拟输入
 * @param	None
 * @retval	None
 */
#if defined(USING_INPUT_REGISTER)
void Read_Analog_Io(void *arg)
{
#define CP 0.005378F
#define CQ 0.375224F
    static bool first_flag = false;
    pModbusHandle pd = Modbus_Object;
/*滤波结构需要不断迭代，否则滤波器无法正常工作*/
#if defined(KALMAN)
    KFP hkfp = {
        .Last_Covariance = LASTP,
        .Kg = 0,
        .Now_Covariance = 0,
        .Output = 0,
        .Q = COVAR_Q,
        .R = COVAR_R,
    };
    static KFP pkpf[ADC_DMA_CHANNEL];
#else
    SideParm side = {
        .First_Flag = true,
        .Head = &side.SideBuff[0],
        .SideBuff = {0},
        .Sum = 0,
    };
    static SideParm pside[ADC_DMA_CHANNEL];
#endif
    /*保证仅首次copy*/
    if (!first_flag)
    {
        first_flag = true;
        for (uint16_t ch = 0; ch < ADC_DMA_CHANNEL; ch++)
        {
#if defined(KALMAN)
            memcpy(&pkfp[ch], &hkfp, sizeof(hkfp));
#else
            memcpy(&pside[ch], &side, sizeof(pside));
#endif
        }
    }
    float *pdata = (float *)CUSTOM_MALLOC(ADC_DMA_CHANNEL * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, ADC_DMA_CHANNEL * sizeof(float));

    for (uint16_t ch = 0; ch < ADC_DMA_CHANNEL; ch++)
    { /*获取DAC值*/
        pdata[ch] = CP * Get_AdcValue(ch) + CQ;
        pdata[ch] = (pdata[ch] <= CQ) ? 0 : pdata[ch];
        /*滤波处理*/
#if defined(KALMAN)
        pdata[ch] = kalmanFilter(&pkfp[ch], pdata[ch]);
#else
        pdata[ch] = sidefilter(&pside[ch], pdata[ch]);
#endif
        /*大小端转换*/
        Endian_Swap((uint8_t *)&pdata[ch], 0U, sizeof(float));

#if defined(USING_DEBUG)
        // shellPrint(Shell_Object, "R_AD[%d] = %.3f\r\n", ch, pdata[ch]);
#endif
    }
    if (pd)
    {
        pd->Slave.Reg_Type = InputRegister;
        pd->Slave.Operate = Write;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, INPUT_ANALOG_START_ADDR, (uint8_t *)&pdata, EXTERN_ANALOGIN_MAX * sizeof(float)))
        {
#if defined(USING_DEBUG)
            Debug("Input register write failed!\r\n");
#endif
        }
    }

__exit:
    CUSTOM_FREE(pdata);
}
#endif

/**
 * @brief	模拟量输出
 * @details	STM32F030F4共在io口扩展了8路数字输出
 * @param	None
 * @retval	None
 */
#if defined(USING_HOLD_REGISTER)
void Write_Analog_IO(void *arg)
{
    Dac_Obj dac_object[EXTERN_ANALOGOUT_MAX] = {
        {.Channel = DAC_OUT1, .Mcpxx_Id = Input_A},
        {.Channel = DAC_OUT2, .Mcpxx_Id = Input_B},
        {.Channel = DAC_OUT3, .Mcpxx_Id = Input_Other},
        {.Channel = DAC_OUT4, .Mcpxx_Id = Input_Other},
    };
    pModbusHandle pd = Modbus_Object;

    float *pdata = (float *)CUSTOM_MALLOC(EXTERN_ANALOGOUT_MAX * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, EXTERN_ANALOGOUT_MAX * sizeof(float));
    /*读出保持寄存器*/
    if (pd)
    {
        pd->Slave.Reg_Type = HoldRegister;
        pd->Slave.Operate = Read;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, OUT_ANALOG_START_ADDR, (uint8_t *)&pdata, EXTERN_ANALOGOUT_MAX * sizeof(float)))
        {
#if defined(USING_DEBUG)
            Debug("Hold register read failed!\r\n");
#endif
        }
    }

    for (uint16_t ch = 0; ch < EXTERN_ANALOGOUT_MAX; ch++)
    {
        /*大小端转换*/
        Endian_Swap((uint8_t *)&pdata[ch], 0U, sizeof(float));
#if defined(USING_DEBUG)
        // shellPrint(Shell_Object, "W_AD[%d] = %.3f\r\n", ch, pdata[ch]);
#endif
        Output_Current(&dac_object[ch], pdata[ch]);
    }

__exit:
    CUSTOM_FREE(pdata);
}
#endif
