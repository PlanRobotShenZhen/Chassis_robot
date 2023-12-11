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
 * @file n32g45x_it.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g45x_it.h"
#include "n32g45x.h"
#include "main.h"
#include "bsp.h"
#include "led.h"

/** @addtogroup N32G45X_StdPeriph_Template
 * @{
 */

extern __IO uint32_t CurrDataCounterEnd;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
//void HardFault_Handler(void)
//{
//    /* Go to infinite loop when Hard Fault exception occurs */
//    while (1)
//    {
//    }
//}

/**
 * @brief  This function handles Memory Manage exception.
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 */
void DebugMon_Handler(void)
{
}

///**
// * @brief  This function handles SysTick Handler.
// */
//void SysTick_Handler(void)
//{
//}

/**
 * @brief  This function handles SPI1 interrupt request defined in main.h .
 */
void SPI1_IRQHandler(void)
{
    if (SPI_I2S_GetStatus(SPI_MASTER, SPI_I2S_RNE_FLAG) != RESET)
    {
        SPI_Master_Rx_Buffer = SPI_MASTER->DAT;
        SPI_ReadWriteCycle = 0;
        GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_RESET);
    }

}


void EXTI4_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(EXTI_LINE4))
    {
        EXTI_ClrITPendBit(EXTI_LINE4);
        TIM_ClrIntPendingBit(TIM3, TIM_INT_UPDATE);
        if (GPIO_ReadInputDataBit(CS1_Econ_PORT, CS1_Econ_PIN))
        {
            ultrasonic_t1tig_time=0;
            TIM_SetCnt(TIM3, 0);
            TIM_Enable(TIM3, ENABLE);
        }
        else
        {
            ultrasonic_t1tig_time = TIM_GetCnt(TIM3);
            TIM_Enable(TIM3, DISABLE);
            ultrasonic_t1tig = 1;
            UltrasonicSetEnable(1, 0);
        }
    }
}
void EXTI9_5_IRQHandler(void)
{
    if (RESET != EXTI_GetITStatus(EXTI_LINE5))
    {
        EXTI_ClrITPendBit(EXTI_LINE5);
        if (GPIO_ReadInputDataBit(CS2_Econ_PORT, CS2_Econ_PIN))
        {
            ultrasonic_t2tig_time = 0;
            TIM_SetCnt(TIM4, 0);
            TIM_Enable(TIM4, ENABLE);
        }
        else
        {
            TIM_Enable(TIM4, DISABLE);
            UltrasonicSetEnable(2, 0);
            ultrasonic_t2tig = 1;
        }
    }
}

/**
 * @brief  This function handles TIM3 update interrupt request.
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM3, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM3, TIM_INT_UPDATE);
        //ultrasonic_t1tig_time++;
    }
}
/**
 * @brief  This function handles TIM4 update interrupt request.
 */
void TIM4_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM4, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM4, TIM_INT_UPDATE);
        ultrasonic_t2tig_time++;
    }
}