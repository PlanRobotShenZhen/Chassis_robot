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
#if(PLAN_CONTROL_BOARD_V==11||PLAN_CONTROL_BOARD_V==12)
void ExioInit(void)
{
    EXIO_INPUT in;
    EXIO_OUTPUT out;
    SPI_InitType SPI_InitStructure;
    NVIC_InitType NVIC_InitStruct;
    GPIO_InitType GPIO_InitStructure;
    /* Enable SPI_MASTER clock and GPIO clock for SPI_MASTER and SPI_SLAVE */
    RCC_EnableAPB2PeriphClk(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK, ENABLE);
    /* Configure SPI_MASTER pins: SCK  MOSI MISO*/
    GPIO_InitStructure.Pin = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI | SPI_MASTER_PIN_MISO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = SPI_MASTER_PIN_NSS;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);
    GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_RESET);

    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.NSS = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_64;
    SPI_InitStructure.FirstBit = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    SPI_I2S_EnableInt(SPI1, SPI_I2S_INT_RNE, ENABLE);
    SPI_Enable(SPI1, ENABLE);
}

#endif

void UltrasonicSetEnable(int id,uint8_t en)
{
    NVIC_InitType NVIC_InitStructure;
    IRQn_Type irq_type = id == 1 ? EXTI4_IRQn : EXTI9_5_IRQn;
    FunctionalState enable = en == 1 ? ENABLE : DISABLE;
    NVIC_InitStructure.NVIC_IRQChannel = irq_type;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = enable;
    NVIC_Init(&NVIC_InitStructure);
}
//< 超声波初始化
void UltrasonicInit(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* TIM1, TIM3 and TIM4 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3 | RCC_APB1_PERIPH_TIM4, ENABLE);
    RCC_EnableAPB2PeriphClk(CS1_Ttig_RCC| CS2_Ttig_RCC, ENABLE);
    /*超声波初始化*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.Pin = CS1_Ttig_PIN;
    GPIO_InitPeripheral(CS1_Ttig_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CS2_Ttig_PIN;
    GPIO_InitPeripheral(CS2_Ttig_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.Pin = CS1_Econ_PIN;
    GPIO_InitPeripheral(CS1_Econ_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CS2_Econ_PIN;
    GPIO_InitPeripheral(CS2_Econ_PORT, &GPIO_InitStructure);

    /*Configure key EXTI Line to key input Pin*/
    //GPIO_ConfigEXTILine(GPIOB_PORT_SOURCE, GPIO_PIN_SOURCE4);
    //GPIO_ConfigEXTILine(GPIOB_PORT_SOURCE, GPIO_PIN_SOURCE5);
    //EXTI_InitStructure.EXTI_Line = EXTI_LINE4;
    //EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    //EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    //EXTI_InitPeripheral(&EXTI_InitStructure);
    //EXTI_InitStructure.EXTI_Line = EXTI_LINE5;
    //EXTI_InitPeripheral(&EXTI_InitStructure);

    //UltrasonicSetEnable(1, 0);
    //UltrasonicSetEnable(2, 0);
    GPIO_ResetBits(CS1_Ttig_PORT, CS1_Ttig_PIN);
    GPIO_ResetBits(CS2_Ttig_PORT, CS2_Ttig_PIN);

    /* Enable the TIM3,4 global Interrupt */
    //NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    //NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

    //TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    //TIM_TimeBaseStructure.Period = 0xfff0;
    //TIM_TimeBaseStructure.Prescaler = 64;
    //TIM_TimeBaseStructure.RepetCnt = 0;
    //TIM_TimeBaseStructure.ClkDiv = TIM_CLK_DIV1;
    //TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
    //TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
    //TIM_InitTimeBase(TIM4, &TIM_TimeBaseStructure);

    /* TIM1 enable update irq */
    //TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);
    //TIM_ConfigInt(TIM4, TIM_INT_UPDATE, ENABLE);

}

void LED_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    NVIC_InitType NVIC_InitStruct;
    EXTI_InitType EXTI_InitStructure;

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
#elif(PLAN_CONTROL_BOARD_V==11||PLAN_CONTROL_BOARD_V==12)

    RCC_EnableAPB2PeriphClk(LED1_PORT_RCC| LED2_PORT_RCC | JDQ_PORT_RCC, ENABLE);


    GPIO_InitStructure.Pin = RUN1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = RUN2;
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = JDQ1_PIN | JDQ2_PIN;
    GPIO_InitPeripheral(JDQ_PORT, &GPIO_InitStructure);

    GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
    ExioInit();
    UltrasonicInit();
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

