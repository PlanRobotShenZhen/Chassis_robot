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
 * @file led.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <rtthread.h>
#include <n32g45x.h>
#include "led.h"

/**
 * @brief  Configures LED GPIO.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOE)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE);
    }
    else if (GPIOx == GPIOF)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
    }
    else
    {
        if (GPIOx == GPIOG)
        {
            RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOG, ENABLE);
        }
    }

    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStructure.Pin        = Pin;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}
void LED_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

#if(PLAN_CONTROL_BOARD_V==10)
    RCC_EnableAPB2PeriphClk(LED1_PORT_RCC | JDQ_PORT_RCC, ENABLE);

    GPIO_InitStructure.Pin = LED_R | LED_G | LED_B | LED_Battery | RUN1 | RUN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED1_PORT, LED_Battery);
    GPIO_SetBits(LED1_PORT, LED_R | LED_G | LED_B);

    GPIO_InitStructure.Pin = JDQ1_PIN | JDQ2_PIN;
    GPIO_InitPeripheral(JDQ_PORT, &GPIO_InitStructure);
    GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
    rt_thread_delay(20000);   //< 2s
    GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
    rt_thread_delay(5000);   //< 500ms
#elif(PLAN_CONTROL_BOARD_V==11)

    RCC_EnableAPB2PeriphClk(LED1_PORT_RCC| LED2_PORT_RCC | JDQ_PORT_RCC, ENABLE);

    GPIO_InitStructure.Pin = RUN1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = RUN2;
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStructure);
    //GPIO_ResetBits(LED_PORT, LED_Battery);
    //GPIO_SetBits(LED_PORT, LED_R | LED_G | LED_B);

    GPIO_InitStructure.Pin = JDQ1_PIN | JDQ2_PIN;
    GPIO_InitPeripheral(JDQ_PORT, &GPIO_InitStructure);
    GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
    rt_thread_delay(20000);   //< 2s
    GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
    rt_thread_delay(5000);   //< 500ms

#endif
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

