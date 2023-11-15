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
 * @file led.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __LED_H__
#define __LED_H__

#include <rtthread.h>
#include "n32g45x.h"

#define LED1_PIN    GPIO_PIN_5
#define LED2_PIN    GPIO_PIN_8

#define LED1_PORT   GPIOB
#define LED2_PORT   GPIOA

#define LED_PORT_RCC     RCC_APB2_PERIPH_GPIOA
#define LED_PORT     GPIOA
#define LED_R        GPIO_PIN_3
#define LED_G        GPIO_PIN_4
#define LED_B        GPIO_PIN_5
#define LED_Battery  GPIO_PIN_6
#define RUN1         GPIO_PIN_1
#define RUN2         GPIO_PIN_2

#define JDQ_PORT_RCC  RCC_APB2_PERIPH_GPIOC
#define JDQ_PORT     GPIOC   
#define JDQ1_PIN    GPIO_PIN_13
#define JDQ2_PIN    GPIO_PIN_14

void rt_hw_led_on(rt_uint32_t led);
void rt_hw_led_off(rt_uint32_t led);
void LED_Init(void);
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);

#endif
