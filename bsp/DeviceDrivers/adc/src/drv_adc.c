/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file drv_adc.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "drv_adc.h"
#include "rt_config.h"
#include "n32g45x.h"
#include "drv_gpio.h"

#ifdef RT_USING_ADC

#if !defined(RT_USING_ADC1) && !defined(RT_USING_ADC2) && !defined(RT_USING_ADC3) && !defined(RT_USING_ADC4)
    #error "Please define at least one RT_USING_ADCx"
    /* this driver can be disabled at menuconfig -> Hardware Drivers Config -> On-chip Peripheral Drivers -> Enable ADC */
#endif


static struct n32g45x_adc_config adc_config[] =
{
#ifdef RT_USING_ADC1
    {
        "adc1",
        ADC1,
    },
#endif
    
#ifdef RT_USING_ADC2
    {
        "adc2",
        ADC2,
    },
#endif
    
#ifdef RT_USING_ADC3
    {
        "adc3",
        ADC3,
    },
#endif
    
#ifdef RT_USING_ADC4
    {
        "adc4",
        ADC4,
    },
#endif
};

static struct n32g45x_adc adc_obj[sizeof(adc_config) / sizeof(adc_config[0])] = {0};

static void n32g45x_adc_init(struct n32g45x_adc_config *config)
{
    ADC_InitType ADC_InitStructure;
    /* ADC configuration ------------------------------------------------------*/
    ADC_InitStructure.MultiChEn      = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init((ADC_Module*)config->adc_periph, &ADC_InitStructure);
    
    /* Enable ADC */
    ADC_Enable((ADC_Module*)config->adc_periph, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatusNew((ADC_Module*)config->adc_periph, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC calibration */
    ADC_StartCalibration((ADC_Module*)config->adc_periph);
    /* Check the end of ADC calibration */
    while (ADC_GetCalibrationStatus((ADC_Module*)config->adc_periph))
        ;
}

static rt_err_t n32g45x_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    if (channel > ADC_CH_18)
    {
        return RT_EINVAL;
    }
    return RT_EOK;
}

static rt_err_t n32g45x_adc_convert(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    struct n32g45x_adc_config *config;
    RT_ASSERT(device != RT_NULL);

    if (channel > ADC_CH_18)
    {
        return RT_EINVAL;
    }
    config = (struct n32g45x_adc_config *)(device->parent.user_data);
    
    ADC_ConfigRegularChannel((ADC_Module*)config->adc_periph, channel, 1, ADC_SAMP_TIME_239CYCLES5);
    
    ADC_EnableSoftwareStartConv((ADC_Module*)config->adc_periph, ENABLE);

    while(ADC_GetFlagStatus((ADC_Module*)config->adc_periph,ADC_FLAG_ENDC)==0)
    {
    }
    ADC_ClearFlag((ADC_Module*)config->adc_periph,ADC_FLAG_ENDC);
    ADC_ClearFlag((ADC_Module*)config->adc_periph,ADC_FLAG_STR);
    *value=ADC_GetDat((ADC_Module*)config->adc_periph);
    
    return RT_EOK;
}

static struct rt_adc_ops n32g45x_adc_ops =
{
    .enabled = n32g45x_adc_enabled,
    .convert = n32g45x_adc_convert,
};

int rt_hw_adc_init(void)
{
    int i = 0;
    int result = RT_EOK;

#if defined(RT_USING_ADC1)
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1, ENABLE);
    /* Configure PC.00 PC.01 as analog input -------------------------*/
    GPIOInit(GPIOC, GPIO_Mode_AIN, GPIO_Speed_50MHz, GPIO_PIN_0 | GPIO_PIN_1);
#endif /* RT_USING_ADC1 */
    
#if defined(RT_USING_ADC2)
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC2, ENABLE);
    /* Configure PC.02 PC.03 as analog input -------------------------*/
    GPIOInit(GPIOC, GPIO_Mode_AIN, GPIO_Speed_50MHz, GPIO_PIN_2 | GPIO_PIN_3);
    #endif /* RT_USING_ADC2 */
    
#if defined(RT_USING_ADC3)
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC3, ENABLE);
    /* Configure PD.10 PD.11 as analog input -------------------------*/
    GPIOInit(GPIOD, GPIO_Mode_AIN, GPIO_Speed_50MHz, GPIO_PIN_10 | GPIO_PIN_11);
#endif /* RT_USING_ADC3 */

#if defined(RT_USING_ADC4)
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC4, ENABLE);
    /* Configure PD.12 PD.13 as analog input -------------------------*/
    GPIOInit(GPIOD, GPIO_Mode_AIN, GPIO_Speed_50MHz, GPIO_PIN_12 | GPIO_PIN_13);
#endif /* RT_USING_ADC4 */
    
    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    
    for (i = 0; i < sizeof(adc_obj) / sizeof(adc_obj[0]); i++)
    {
        adc_obj[i].config = &adc_config[i];
        n32g45x_adc_init(&adc_config[i]);
        rt_hw_adc_register(&adc_obj[i].adc_device, \
                           adc_obj[i].config->name, &n32g45x_adc_ops, adc_obj[i].config);
    }
    return result;

}

INIT_DEVICE_EXPORT(rt_hw_adc_init);

#endif /* RT_USING_ADC */

