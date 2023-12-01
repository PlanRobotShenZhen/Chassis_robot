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
 * @file bsp.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_H__
#define __BSP_H__


//  <i>Default: 144
#define N32G45X_SRAM_SIZE           144
#define N32G45X_SRAM_START          (0x20000000 + N32G45X_SRAM_SIZE/2 * 1024)
#define N32G45X_SRAM_END            (0x20000000 + N32G45X_SRAM_SIZE * 1024)

#define MEMORY_ADDR_FIRST			0x0807F800	//数据存储区首地址

typedef union __SPI_IO_INPUT
{
	struct {
		unsigned char X0 : 1;
		unsigned char X1 : 1;
		unsigned char X2 : 1;
		unsigned char X3 : 1;
		unsigned char X4 : 1;
		unsigned char X5 : 1;
		unsigned char X6 : 1;
		unsigned char X7 : 1;
	}bit;
	unsigned char input;
}EXIO_INPUT;

typedef struct __SPI_IO_OUTPUT
{
	struct {
		unsigned char RGB_G : 1;
		unsigned char RGB_B : 1;
		unsigned char RGB_R : 1;
		unsigned char Light : 1;
		unsigned char Light_Q : 1;
		unsigned char Light_Z : 1;
		unsigned char Light_Y : 1;
		unsigned char Light_H : 1;
	}bit;
	unsigned char output;
}EXIO_OUTPUT;

#define PLAN_CONTROL_BOARD_V 11
#if(PLAN_CONTROL_BOARD_V==10)
#define PLAN_CONTROL_BOARD_VERSION "V1.0"

#define LED1_PORT_RCC     RCC_APB2_PERIPH_GPIOA
#define LED2_PORT_RCC     RCC_APB2_PERIPH_GPIOA
#define LED1_PORT     GPIOA
#define LED2_PORT     GPIOA

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



/*USART1 */
#define USARTy            USART1
#define USARTy_GPIO       GPIOA
#define USARTy_CLK        RCC_APB2_PERIPH_USART1
#define USARTy_GPIO_CLK   RCC_APB2_PERIPH_GPIOA
#define USARTy_RxPin      GPIO_PIN_10
#define USARTy_TxPin      GPIO_PIN_9
#define USARTy_DMAx_CLK       RCC_AHB_PERIPH_DMA1
#define USARTy_APBxClkCmd RCC_EnableAPB2PeriphClk
#define USARTy_IRQHandler USART1_IRQHandler
#define USARTy_IRQn           USART1_IRQn
#define USARTy_Tx_DMA_Channel DMA1_CH4
#define USARTy_Rx_DMA_Channel DMA1_CH5
#define USARTy_DR_Base        (USART1_BASE + 0x04)
#define USARTy_Tx_DMA_FLAG    DMA1_FLAG_TC4

/*USART3 */
#define USARTz					USART3
#define USARTz_GPIO				GPIOB
#define USARTz_CLK				RCC_APB1_PERIPH_USART3
#define USARTz_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin			GPIO_PIN_11
#define USARTz_TxPin			GPIO_PIN_10
#define USARTz_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTz_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTz_IRQHandler		USART3_IRQHandler
#define USARTz_IRQn				USART3_IRQn
#define USARTz_Tx_DMA_Channel	DMA1_CH2
#define USARTz_Rx_DMA_Channel	DMA1_CH3
#define USARTz_DR_Base			(USART3_BASE + 0x04)
#define USARTz_Tx_DMA_FLAG		DMA1_FLAG_TC2
/*USART5 */
#define USARTe					UART5
#define USARTe_GPIO				GPIOB
#define USARTe_CLK				RCC_APB1_PERIPH_UART5
#define USARTe_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define USARTe_RxPin			GPIO_PIN_14
//#define USARTe_TxPin			GPIO_PIN_10
#define USARTe_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTe_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTe_IRQHandler		UART5_IRQHandler
#define USARTe_IRQn				UART5_IRQn
//#define USARTe_Tx_DMA_Channel	DMA1_CH2
#define USARTe_Rx_DMA_Channel	DMA1_CH8
#define USARTe_DR_Base			(UART5_BASE + 0x04)
#define USARTe_Tx_DMA_FLAG		DMA1_FLAG_TC2

/*CAN1 */

#define CANa_GPIO		    GPIOB
#define CANa_RxPin			GPIO_PIN_8
#define CANa_TxPin			GPIO_PIN_9
/*CAN2 */
#define CANb_GPIO		    GPIOB
#define CANb_RxPin			GPIO_PIN_12
#define CANb_TxPin			GPIO_PIN_13

#elif(PLAN_CONTROL_BOARD_V==11)
#define PLAN_CONTROL_BOARD_VERSION "V1.1"
#define LED1_PORT_RCC     RCC_APB2_PERIPH_GPIOC
#define LED2_PORT_RCC     RCC_APB2_PERIPH_GPIOB
#define LED1_PORT     GPIOC
#define LED2_PORT     GPIOB
//#define LED_R        GPIO_PIN_3
//#define LED_G        GPIO_PIN_4
//#define LED_B        GPIO_PIN_5
#define RUN1         GPIO_PIN_15
#define RUN2         GPIO_PIN_6

#define JDQ_PORT_RCC  RCC_APB2_PERIPH_GPIOC
#define JDQ_PORT     GPIOC   
#define JDQ1_PIN    GPIO_PIN_13
#define JDQ2_PIN    GPIO_PIN_14


#define SPI1_PORT        GPIOA
#define SPI1_NSS_PIN     GPIO_PIN_4
#define SPI1_SCK_PIN     GPIO_PIN_5
#define SPI1_MISO_PIN    GPIO_PIN_6
#define SPI1_MOSI_PIN    GPIO_PIN_7


/*USART1 */
#define USARTy            USART1
#define USARTy_GPIO       GPIOA
#define USARTy_CLK        RCC_APB2_PERIPH_USART1
#define USARTy_GPIO_CLK   RCC_APB2_PERIPH_GPIOA
#define USARTy_RxPin      GPIO_PIN_10
#define USARTy_TxPin      GPIO_PIN_9
#define USARTy_DMAx_CLK       RCC_AHB_PERIPH_DMA1
#define USARTy_APBxClkCmd RCC_EnableAPB2PeriphClk
#define USARTy_IRQHandler USART1_IRQHandler
#define USARTy_IRQn           USART1_IRQn
#define USARTy_Tx_DMA_Channel DMA1_CH4
#define USARTy_Rx_DMA_Channel DMA1_CH5
#define USARTy_DR_Base        (USART1_BASE + 0x04)
#define USARTy_Tx_DMA_FLAG    DMA1_FLAG_TC4

/*USART3 */
#define USARTz					USART3
#define USARTz_GPIO				GPIOB
#define USARTz_CLK				RCC_APB1_PERIPH_USART3
#define USARTz_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin			GPIO_PIN_11
#define USARTz_TxPin			GPIO_PIN_10
#define USARTz_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTz_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTz_IRQHandler		USART3_IRQHandler
#define USARTz_IRQn				USART3_IRQn
#define USARTz_Tx_DMA_Channel	DMA1_CH2
#define USARTz_Rx_DMA_Channel	DMA1_CH3
#define USARTz_DR_Base			(USART3_BASE + 0x04)
#define USARTz_Tx_DMA_FLAG		DMA1_FLAG_TC2
/*USART5 */
#define USARTe					UART5
#define USARTe_GPIO				GPIOB
#define USARTe_CLK				RCC_APB1_PERIPH_UART5
#define USARTe_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define USARTe_RxPin			GPIO_PIN_14
//#define USARTe_TxPin			GPIO_PIN_10
#define USARTe_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTe_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTe_IRQHandler		UART5_IRQHandler
#define USARTe_IRQn				UART5_IRQn
//#define USARTe_Tx_DMA_Channel	DMA1_CH2
#define USARTe_Rx_DMA_Channel	DMA1_CH8
#define USARTe_DR_Base			(UART5_BASE + 0x04)
#define USARTe_Tx_DMA_FLAG		DMA1_FLAG_TC2

/*CAN1 */
#define CANa_GPIO		    GPIOB
#define CANa_RxPin			GPIO_PIN_8
#define CANa_TxPin			GPIO_PIN_9
/*CAN2 */
#define CANb_GPIO		    GPIOB
#define CANb_RxPin			GPIO_PIN_12
#define CANb_TxPin			GPIO_PIN_13


/*ADC1_JT*/
#define ADC1_JT_GPIO		GPIOA
#define ADC1_JT_RxPin		GPIO_PIN_0
#define ADC1_JT_ADC		    ADC1
#define ADC1_JT_ADC_Channel	ADC1_Channel_01_PA0

/*ADC2*/
#define ADC2_GPIO		    GPIOB
#define ADC2_RxPin			GPIO_PIN_1
#define ADC2_ADC		    ADC2
#define ADC2_ADC_Channel	ADC2_Channel_03_PB1

/*YL_1*/
#define YL_1_GPIO		    GPIOA
#define YL_1_RxPin			GPIO_PIN_1
#define YL_1_JT_ADC		    ADC1
#define YL_1_JT_ADC_Channel	ADC1_Channel_02_PA1

/*YL_2*/
#define YL_2_GPIO		    GPIOA
#define YL_2_RxPin			GPIO_PIN_2
#define YL_2_JT_ADC		    ADC1
#define YL_2_JT_ADC_Channel	ADC1_Channel_11_PA2

/*YL_3*/
#define YL_3_GPIO		    GPIOA
#define YL_3_RxPin			GPIO_PIN_3
#define YL_3_JT_ADC		    ADC1
#define YL_3_JT_ADC_Channel	ADC1_Channel_04_PA3

/*YL_4*/
#define YL_4_GPIO		    GPIOB
#define YL_4_RxPin			GPIO_PIN_0
#define YL_4_JT_ADC		    ADC3
#define YL_4_JT_ADC_Channel	ADC3_Channel_12_PB0

/*YL_5*/
#define YL_5_GPIO		    GPIOB
#define YL_5_RxPin			GPIO_PIN_15
#define YL_5_JT_ADC		    ADC4
#define YL_5_JT_ADC_Channel	ADC4_Channel_05_PB15

#endif



void rt_hw_board_init(void);

#define PRINT_RCC_FREQ_INFO

#endif

