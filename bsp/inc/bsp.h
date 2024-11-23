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

#include "stdint.h"
//  <i>Default: 144
            
#define Usknow		-1
#define Mec			0           // 麦轮小车
#define Omni		2           // 万向轮小车
#define Akm			2           // 阿克曼
#define Diff		3			// 差速车(圆形底盘)
#define FourWheel	4			// 室外差速四驱车
#define TwoWheel	5			// 室内差速二驱车
#define Tank		6           // 坦克车
#define RC          7		    // 竞赛小车
#define Chrg		8           // 充电桩

#define CARMODE						Diff
#define TIM 							50	//< 5ms
#define DELAY_COUNT_1S 		(1000 / (TIM / 10))
#define N32G45X_SRAM_SIZE           144
#define N32G45X_SRAM_START          (0x20000000 + N32G45X_SRAM_SIZE/2 * 1024)
#define N32G45X_SRAM_END            (0x20000000 + N32G45X_SRAM_SIZE * 1024)

#define MEMORY_ADDR_FIRST			0x0807F800	//数据存储区首地址

typedef union __SPI_IO_INPUT
{
	struct {
		unsigned char X0 : 1;//《 急停开关
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

typedef union __SPI_IO_OUTPUT
{
	struct {
		unsigned char RGB_R : 1;
		unsigned char Buzzer : 1;		//< 给蜂鸣器用
		unsigned char Front_Red : 1;
		unsigned char RGB_G : 1;	
		unsigned char RR_White : 1;		//< 右后
		unsigned char RF_White : 1;		//< 右前
		unsigned char LR_White : 1;		//< 左后
		unsigned char LF_White : 1;		//< 左前
	}bit;
	unsigned char output;
}EXIO_OUTPUT;

typedef struct {
	uint32_t t_cnt_RGB_R;
	uint32_t t_cnt_Buzzer;
	uint32_t t_cnt_Front_Red;
	uint32_t t_cnt_RGB_G;
	uint32_t t_cnt_RR_White;	//< 右后
	uint32_t t_cnt_RF_White;	//< 右前
	uint32_t t_cnt_LR_White;	//< 左后
	uint32_t t_cnt_LF_White;	//< 左前
	uint32_t t_cnt_Light_ALL;	//< 所有灯
}LightTime;
extern LightTime light_time;

extern EXIO_INPUT exio_input;
extern EXIO_OUTPUT exio_output;

#if(CARMODE == Diff)
/*圆形底盘小车引脚定义采用新版本*/
#define PLAN_CONTROL_BOARD_V 13
#else
#define PLAN_CONTROL_BOARD_V 12
#endif

#if(PLAN_CONTROL_BOARD_V==11)
#define PLAN_CONTROL_BOARD_VERSION "V1.1"
#endif
#if(PLAN_CONTROL_BOARD_V==12)
#define PLAN_CONTROL_BOARD_VERSION "V1.2"

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

#define CS1_Ttig_RCC     RCC_APB2_PERIPH_GPIOA
#define CS1_Ttig_PORT     GPIOA
#define CS1_Ttig_PIN     GPIO_PIN_15
#define CS1_Econ_PORT     GPIOB
#define CS1_Econ_PIN     GPIO_PIN_4

#define CS2_Ttig_RCC     RCC_APB2_PERIPH_GPIOB
#define CS2_Ttig_PORT     GPIOB
#define CS2_Ttig_PIN     GPIO_PIN_3
#define CS2_Econ_PORT     GPIOB
#define CS2_Econ_PIN     GPIO_PIN_5


/*USART1 */
#define USARTOne            		USART1
#define USARTOne_GPIO       		GPIOA
#define USARTOne_CLK        		RCC_APB2_PERIPH_USART1
#define USARTOne_GPIO_CLK   		RCC_APB2_PERIPH_GPIOA
#define USARTOne_RxPin      		GPIO_PIN_10
#define USARTOne_TxPin      		GPIO_PIN_9
#define USARTOne_DMAx_CLK       	RCC_AHB_PERIPH_DMA1
#define USARTOne_APBxClkCmd 		RCC_EnableAPB2PeriphClk
#define USARTOne_IRQHandler 		USART1_IRQHandler
#define USARTOne_IRQn           	USART1_IRQn
#define USARTOne_Tx_DMA_Channel 	DMA1_CH4
#define USARTOne_Rx_DMA_Channel 	DMA1_CH5
#define USARTOne_DR_Base        	(USART1_BASE + 0x04)
#define USARTOne_Tx_DMA_FLAG    	DMA1_FLAG_TC4

/*USART3 */
#define USARTThree					USART3
#define USARTThree_GPIO				GPIOB
#define USARTThree_CLK				RCC_APB1_PERIPH_USART3
#define USARTThree_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define USARTThree_RxPin			GPIO_PIN_11
#define USARTThree_TxPin			GPIO_PIN_10
#define USARTThree_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTThree_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTThree_IRQHandler		USART3_IRQHandler
#define USARTThree_IRQn				USART3_IRQn
#define USARTThree_Tx_DMA_Channel	DMA1_CH2
#define USARTThree_Rx_DMA_Channel	DMA1_CH3
#define USARTThree_DR_Base			(USART3_BASE + 0x04)
#define USARTThree_Tx_DMA_FLAG		DMA1_FLAG_TC2

/*UART4 电池信息读取*/
#define UARTFour					UART4
#define UARTFour_GPIO				GPIOA
#define UARTFour_CLK				RCC_APB1_PERIPH_UART4
#define UARTFour_GPIO_CLK			RCC_APB2_PERIPH_GPIOA
#define UARTFour_RxPin				GPIO_PIN_14
#define UARTFour_TxPin				GPIO_PIN_13
#define UARTFour_485enPin			GPIO_PIN_7
#define UARTFour_485en_GPIO			GPIOB
#define UARTFour_DMAx_CLK			RCC_AHB_PERIPH_DMA2
#define UARTFour_APBxClkCmd			RCC_EnableAPB2PeriphClk
#define UARTFour_IRQHandler			UART4_IRQHandler
#define UARTFour_IRQn				UART4_IRQn
#define UARTFour_Tx_DMA_Channel		DMA2_CH5
#define UARTFour_Rx_DMA_Channel		DMA2_CH3
#define UARTFour_DR_Base			(UART4_BASE + 0x04)
#define UARTFour_Tx_DMA_FLAG		DMA2_FLAG_TC5

/*UART5 */
#define UARTFive					UART5
#define UARTFive_GPIO				GPIOB
#define UARTFive_CLK				RCC_APB1_PERIPH_UART5
#define UARTFive_GPIO_CLK			RCC_APB2_PERIPH_GPIOB
#define UARTFive_RxPin				GPIO_PIN_14
#define UARTFive_TxPin				GPIO_PIN_10
#define UARTFive_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define UARTFive_APBxClkCmd			RCC_EnableAPB2PeriphClk
#define UARTFive_IRQHandler			UART5_IRQHandler
#define UARTFive_IRQn				UART5_IRQn
#define UARTFive_Tx_DMA_Channel		DMA1_CH1
#define UARTFive_Rx_DMA_Channel		DMA1_CH8
#define UARTFive_DR_Base			(UART5_BASE + 0x04)
#define UARTFive_Tx_DMA_FLAG		DMA1_FLAG_TC2

/*CAN1 */
#define CANa_GPIO		    GPIOB
#define CANa_RxPin			GPIO_PIN_8
#define CANa_TxPin			GPIO_PIN_9
/*CAN2 */
#define CANb_GPIO		    GPIOB
#define CANb_RxPin			GPIO_PIN_12
#define CANb_TxPin			GPIO_PIN_13


/*RJ_JT 急停输出 output*/
#define RJ_JT_GPIO			GPIOA
#define RJ_JT_Pin			GPIO_PIN_8
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
#define YL_1_ADC		    ADC1
#define YL_1_ADC_Channel	ADC1_Channel_02_PA1

/*YL_2*/
#define YL_2_GPIO		    GPIOA
#define YL_2_RxPin			GPIO_PIN_2
#define YL_2_ADC		    ADC1
#define YL_2_ADC_Channel	ADC1_Channel_11_PA2

/*YL_3*/
#define YL_3_GPIO		    GPIOA
#define YL_3_RxPin			GPIO_PIN_3
#define YL_3_ADC		    ADC1
#define YL_3_ADC_Channel	ADC1_Channel_04_PA3

/*YL_4*/
#define YL_4_GPIO		    GPIOB
#define YL_4_RxPin			GPIO_PIN_0
#define YL_4_ADC		    ADC3
#define YL_4_ADC_Channel	ADC3_Channel_12_PB0

/*YL_5*/
#define YL_5_GPIO		    GPIOB
#define YL_5_RxPin			GPIO_PIN_15
#define YL_5_ADC		    ADC4
#define YL_5_ADC_Channel	ADC4_Channel_05_PB15
/*YL_5*/
#define YL_6_GPIO		    GPIOA
#define YL_6_Pin			GPIO_PIN_12
/*YL_5*/
#define YL_7_GPIO		    GPIOA
#define YL_7_Pin			GPIO_PIN_11
/*SPI1*/
#define SPI_MASTER                SPI1
#define SPI_MASTER_CLK            RCC_APB2_PERIPH_SPI1
#define SPI_MASTER_GPIO           GPIOA
#define SPI_MASTER_GPIO_CLK       RCC_APB2_PERIPH_GPIOA
#define SPI_MASTER_PIN_SCK        GPIO_PIN_5
#define SPI_MASTER_PIN_NSS        GPIO_PIN_4
#define SPI_MASTER_PIN_MOSI       GPIO_PIN_7
#define SPI_MASTER_PIN_MISO       GPIO_PIN_6

extern unsigned char SPI_Master_Rx_Buffer;
extern unsigned char SPI_ReadWriteCycle;

//充电桩相关参数
#define LED_LEFT 			RR_White
#define LED_RIGHT 			LF_White
#define MCU_INF_TX 			exio_output.bit.RGB_R
#define MCU_RELAY2 			exio_output.bit.Buzzer
#define IrDA_TX 			MCU_INF_TX

//风扇相关参数
#define MCU_FAN1			exio_output.bit.LR_White
#define MCU_FAN2			exio_output.bit.RF_White
#define FAN1				MCU_FAN1
#define FAN2				MCU_FAN2

#define PRINT_RCC_FREQ_INFO
#elif(PLAN_CONTROL_BOARD_V==13)
#define PLAN_CONTROL_BOARD_VERSION "V1.3"
//圆形底盘

#define LED1_PORT				LED17_GPIO //端口命名统一
#define RUN1					LED17_PIN
#define LED1_PORT_RCC			LED17_CLK
#define LED17_CLK				RCC_APB2_PERIPH_GPIOC
#define LED17_GPIO				GPIOC
#define LED17_PIN				GPIO_PIN_15

#define MCU_RGB_CLK				RCC_APB2_PERIPH_GPIOA
#define MCU_RGB_GPIO			GPIOA
#define MCU_RGB_RED_PIN			GPIO_PIN_1
#define MCU_RGB_GREEN_PIN		GPIO_PIN_2
#define MCU_RGB_BLUE_PIN		GPIO_PIN_3

#define CS1_CLK					RCC_APB2_PERIPH_GPIOA
#define CS1_TIM_CLK				RCC_APB1_PERIPH_TIM3
#define CS1_TTIG_GPIO			GPIOA
#define CS1_TTIG_PIN			GPIO_PIN_6
#define CS1_ECON_GPIO			GPIOA
#define CS1_ECON_PIN			GPIO_PIN_7
#define CS1_ECON_TIM			TIM3
#define CS1_ECON_TIM_CCx		TIM_INT_UPDATE
#define CS1_ECON_Channel		TIM_CH_2
#define CS1_ECON_IRQn			TIM3_IRQn	//	EXTI7
#define CS1_ECON_EXTI_LINE		EXTI_LINE7

#define CS2_CLK						RCC_APB2_PERIPH_GPIOB
#define CS2_TIM_CLK				RCC_APB1_PERIPH_TIM3 
#define CS2_TTIG_GPIO			GPIOB
#define CS2_TTIG_PIN			GPIO_PIN_0
#define CS2_ECON_GPIO			GPIOB
#define CS2_ECON_PIN			GPIO_PIN_1
#define CS2_ECON_TIM			TIM3
#define CS2_ECON_Channel	TIM_CH_4
#define CS2_ECON_IRQn			TIM3_IRQn
#define CS2_ECON_EXTI_LINE		EXTI_LINE1

/*USART1 USB1*/
#define USB1					USART1
#define USARTOne				USART1
#define USARTOne_GPIO			GPIOA
#define USARTOne_CLK			RCC_APB2_PERIPH_USART1
#define USARTOne_GPIO_CLK		RCC_APB2_PERIPH_GPIOA
#define USARTOne_TxPin			GPIO_PIN_9
#define USARTOne_RxPin			GPIO_PIN_10
#define USARTOne_DMAx_CLK		RCC_AHB_PERIPH_DMA1
#define USARTOne_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTOne_IRQHandler		USART1_IRQHandler
#define USARTOne_IRQn			USART1_IRQn
#define USARTOne_Tx_DMA_Channel	DMA1_CH4
#define USARTOne_Rx_DMA_Channel	DMA1_CH5
#define USARTOne_DR_Base		(USART1_BASE + 0x04)
#define USARTOne_Tx_DMA_FLAG	DMA1_FLAG_TC4

/*USART2 RS485*/
//引脚统一命名
#define UARTFour_485en_GPIO 		USARTTwo_GPIO
#define UARTFour_485enPin			USARTTwo_CTS
#define UARTFour_Tx_DMA_Channel		USARTTwo_Tx_DMA_Channel
#define UARTFour_Rx_DMA_Channel		USARTTwo_Rx_DMA_Channel

#define RS485						USART2
#define USARTTwo					USART2
#define USARTTwo_GPIO				GPIOC
#define USARTTwo_CLK				RCC_APB1_PERIPH_USART2
#define USARTTwo_GPIO_CLK			RCC_APB2_PERIPH_GPIOC
#define USARTTwo_RxPin				GPIO_PIN_9
#define USARTTwo_TxPin				GPIO_PIN_8
#define USARTTwo_CTS				GPIO_PIN_6 //目前用它代替485en引脚
#define USARTTwo_RTS				GPIO_PIN_7
#define USARTTwo_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTTwo_IRQn				USART2_IRQn
#define USARTTwo_IRQHandler			USART2_IRQHandler
#define USARTTwo_Tx_DMA_Channel		DMA1_CH7
#define USARTTwo_Rx_DMA_Channel		DMA1_CH6
#define USARTTwo_DR_Base			(USART2_BASE + 0x04)
#define USARTTwo_Tx_DMA_FLAG		DMA1_FLAG_TC5

/*USART3 RS232*/
#define RS232						USART3
#define USARTThree					USART3
#define USARTThree_GPIO				GPIOC
#define USARTThree_CLK				RCC_APB1_PERIPH_USART3
#define USARTThree_GPIO_CLK			RCC_APB2_PERIPH_GPIOC
#define USARTThree_RxPin			GPIO_PIN_11
#define USARTThree_TxPin			GPIO_PIN_10
#define USARTThree_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define USARTThree_APBxClkCmd		RCC_EnableAPB2PeriphClk
#define USARTThree_IRQHandler		USART3_IRQHandler
#define USARTThree_IRQn				USART3_IRQn
#define USARTThree_Tx_DMA_Channel	DMA1_CH2
#define USARTThree_Rx_DMA_Channel	DMA1_CH3
#define USARTThree_DR_Base			(USART3_BASE + 0x04)
#define USARTThree_Tx_DMA_FLAG		DMA1_FLAG_TC2


/*USART5 遥控器接收*/
#define UARTFive					UART5
#define UARTFive_GPIO				GPIOD
#define UARTFive_CLK				RCC_APB1_PERIPH_UART5
#define UARTFive_GPIO_CLK			RCC_APB2_PERIPH_GPIOD
#define UARTFive_RxPin				GPIO_PIN_2
//#define USARTe_TxPin				GPIO_PIN_10
#define UARTFive_DMAx_CLK			RCC_AHB_PERIPH_DMA1
#define UARTFive_APBxClkCmd			RCC_EnableAPB2PeriphClk
#define UARTFive_IRQHandler			UART5_IRQHandler
#define UARTFive_IRQn				UART5_IRQn
//#define USARTe_Tx_DMA_Channel	DMA1_CH2
#define UARTFive_Rx_DMA_Channel		DMA1_CH8
#define UARTFive_DR_Base			(UART5_BASE + 0x04)
#define UARTFive_Tx_DMA_FLAG		DMA1_FLAG_TC2

/*CAN2 电机驱动控制*/
#define CANb_CLK			RCC_APB2_PERIPH_GPIOB
#define CANb_GPIO		    GPIOB
#define CANb_RxPin			GPIO_PIN_12
#define CANb_TxPin			GPIO_PIN_13

/*RJ_JT 软急停控制输出 output*/
//此软急停为GPIO口控制，用于后续ros等上位机调用
#define RJ_JT_GPIO 				ESTOP_SOFT_GPIO //圆形底盘与其他车型统一名称
#define RJ_JT_Pin					ESTOP_SOFT_PIN
#define ESTOP_SOFT_CLK		RCC_APB2_PERIPH_GPIOB
#define ESTOP_SOFT_GPIO		GPIOB
#define ESTOP_SOFT_PIN		GPIO_PIN_3

/*ADC2*/
//电机电源电流采样
#define MCU_MT_CURR_ADC					ADC2
#define MCU_MT_CURR_ADC_GPIO		    GPIOC
#define MCU_MT_CURR_ADC_RxPin			GPIO_PIN_4
#define MCU_MT_CURR_ADC_Channel			ADC2_Channel_05_PC4
//24V输出电流采样
#define MCU_ADC_24VARM_IOUT				ADC2
#define MCU_ADC_24VARM_IOUT_GPIO	    GPIOA
#define MCU_ADC_24VARM_IOUT_RxPin		GPIO_PIN_4
#define MCU_ADC_24VARM_IOUT_Channel		ADC2_Channel_01_PA4
//19V输出电流采样
#define MCU_ADC_19V_IOUT				ADC2
#define MCU_ADC_19V_IOUT_GPIO		    GPIOC
#define MCU_ADC_19V_IOUT_RxPin			GPIO_PIN_5
#define MCU_ADC_19V_IOUT_Channel		ADC2_Channel_12_PC5
//5V输出电流采样
#define MCU_ADC_5V_IOUT					ADC2
#define MCU_ADC_5V_IOUT_GPIO		    GPIOA
#define MCU_ADC_5V_IOUT_RxPin			GPIO_PIN_5
#define MCU_ADC_5V_IOUT_Channel			ADC2_Channel_02_PA5
/*ADC4*/
//12V输出电流采样
#define MCU_ADC_12VPC_IOUT				ADC4
#define MCU_ADC_12VPC_IOUT_GPIO		    GPIOB
#define	MCU_ADC_12VPC_IOUT_RxPin		GPIO_PIN_15
#define	MCU_ADC_12VPC_IOUT_Channel		ADC4_Channel_05_PB15

/*自动充电控制*/
#define MCU_CHARGE_ON_CLK				RCC_APB2_PERIPH_GPIOC
#define MCU_CHARGE_ON_GPIO				GPIOC
#define MCU_CHARGE_ON_PIN				GPIO_PIN_12

/*19V电源使能控制*/
#define MCU_19VARM_PWR_ON_CLK			RCC_APB2_PERIPH_GPIOA
#define MCU_19VARM_PWR_ON_GPIO			GPIOA
#define MCU_19VARM_PWR_ON_PIN			GPIO_PIN_15

/*24V电源使能控制*/
#define MCU_24VARM_PWR_ON_CLK			RCC_APB2_PERIPH_GPIOA
#define MCU_24VARM_PWR_ON_GPIO			GPIOA
#define MCU_24VARM_PWR_ON_PIN			GPIO_PIN_8

/*电机电源使能控制*/
#define MCU_MT_PWR_ON_CLK				RCC_APB2_PERIPH_GPIOB
#define MCU_MT_PWR_ON_GPIO				GPIOB
#define MCU_MT_PWR_ON_PIN				GPIO_PIN_14

/*释放急停开关检测（解抱闸）*/
#define UNESTOP_SW_IN_CLK				RCC_APB2_PERIPH_GPIOC
#define UNESTOP_SW_IN_GPIO				GPIOC
#define UNESTOP_SW_IN_PIN				GPIO_PIN_13

/*急停开关检测*/
#define ESTOP_SW_IN_CLK					RCC_APB2_PERIPH_GPIOC
#define ESTOP_SW_IN_GPIO				GPIOC
#define ESTOP_SW_IN_PIN					GPIO_PIN_14

/*红外对管1*/
#define MCU_INF_CLK 					RCC_APB2_PERIPH_GPIOB
#define MCU_INF_TIM_CLK 				RCC_APB1_PERIPH_TIM4

#define MCU_INF_TX1_GPIO 				GPIOB
#define MCU_INF_TX1_PIN 				GPIO_PIN_6
//#define MCU_INF_TX1_TIM 				TIM4
//#define MCU_INF_TX1_TIM_IRQn 			TIM4_IRQn
//#define MCU_INF_TX1_TIM_CHx_Init 		TIM_InitOc1
//#define MCU_INF_TX1_TIM_CHx_Preload 	TIM_ConfigOc1Preload

#define MCU_INF_RX1_GPIO 				GPIOB
#define MCU_INF_RX1_PIN 				GPIO_PIN_7
//#define MCU_INF_RX1_TIM 				TIM4
//#define MCU_INF_RX1_TIM_IRQn 			TIM4_IRQn
//#define MCU_INF_RX1_TIM_CHx_Init 		TIM_InitOc2
//#define MCU_INF_RX1_TIM_CHx_Preload 	TIM_ConfigOc2Preload


/*RGB控制*/
#define MCU_RGB_CLK 					RCC_APB2_PERIPH_GPIOA
#define	MCU_RGB_TIM_CLK 				RCC_APB1_PERIPH_TIM5
#define	MCU_RGB_TIM 					TIM5
#define	MCU_RGB_TIM_IRQn 				TIM5_IRQn
#define MCU_RGB_TIM_Period 				100					// 周期为1ms
#define MCU_RGB_TIM_Prescaler 			720
//red
#define MCU_RGB_RED_GPIO 				GPIOA
#define MCU_RGB_RED_PIN 				GPIO_PIN_1
#define MCU_RGB_RED_TIM_CHx_Init 		TIM_InitOc2
#define MCU_RGB_RED_TIM_CHx_Preload 	TIM_ConfigOc2Preload
#define MCU_RGB_RED_TIM_SetCmp 			TIM_SetCmp2
//green
#define MCU_RGB_GREEN_GPIO 				GPIOA
#define MCU_RGB_GREEN_PIN 				GPIO_PIN_2
#define MCU_RGB_GREEN_TIM_CHx_Init 		TIM_InitOc3
#define MCU_RGB_GREEN_TIM_CHx_Preload	TIM_ConfigOc3Preload
#define MCU_RGB_GREEN_TIM_SetCmp 		TIM_SetCmp3
//blue
#define MCU_RGB_BLUE_GPIO 				GPIOA
#define MCU_RGB_BLUE_PIN 				GPIO_PIN_3
#define MCU_RGB_BLUE_TIM_CHx_Init 		TIM_InitOc4
#define MCU_RGB_BLUE_TIM_CHx_Preload 	TIM_ConfigOc4Preload
#define MCU_RGB_BLUE_TIM_SetCmp 		TIM_SetCmp4
/*转向灯控制*/
#define MCU_LED_CLK 					RCC_APB2_PERIPH_GPIOB
//左
#define MCU_LED_LEFT_GPIO 				GPIOB
#define MCU_LED_LEFT_PIN 				GPIO_PIN_10
#define MCU_LED_LEFT_TIM 				TIM2
#define MCU_LED_LEFT_TIM_IRQn 			TIM2_IRQn
#define MCU_LED_LEFT_TIM_CHx_Init 		TIM_InitOc3
#define MCU_LED_LEFT_TIM_CHx_Preload 	TIM_ConfigOc3Preload
//右
#define MCU_LED_RIGHT_GPIO 				GPIOB
#define MCU_LED_RIGHT_PIN 				GPIO_PIN_11
#define MCU_LED_RIGHT_TIM 				TIM2
#define MCU_LED_RIGHT_TIM_IRQn 			TIM2_IRQn
#define MCU_LED_RIGHT_TIM_CHx_Init 		TIM_InitOc4
#define MCU_LED_RIGHT_TIM_CHx_Preload	TIM_ConfigOc4Preload

#endif


void UltrasonicSetEnable(int id, uint8_t en);
void MY_NVIC_SetVectorTable(unsigned int NVIC_VectTab, unsigned int Offset);//设置偏移地址
void rt_hw_board_init(void);
void rtthread_startup(void);
void rt_application_init(void);

#endif

