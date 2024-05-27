#ifndef __CHARGER_H
#define __CHARGER_H
#include "n32g45x.h"                // Device header
#define MCU_INF_RX_CLK 				RCC_APB2_PERIPH_GPIOA
#define MCU_INF_RX_GPIO 			GPIOA
#define MCU_INF_RX_PIN 				GPIO_PIN_0
#define MCU_INF_RX_TIM_CLK 			RCC_APB1_PERIPH_TIM5
#define MCU_INF_RX_TIM 				TIM5
#define MCU_INF_RX_TIM_IRQn 		TIM5_IRQn
#define MCU_RELAY1					ADC2
#define MCU_RELAY1_CLK				RCC_APB2_PERIPH_GPIOB
#define MCU_RELAY1_GPIO				GPIOB
#define MCU_RELAY1_PIN				GPIO_PIN_1
#define KEY_CLK						RCC_APB2_PERIPH_GPIOA
#define KEY_GPIO					GPIOA
#define KEY_PIN						GPIO_PIN_8
#define RGB_TIM_IRQn 				TIM1_UP_IRQn
#define RGB_TIM_CLK					RCC_APB2_PERIPH_TIM1
#define RGB_CLK						RCC_APB2_PERIPH_GPIOA
#define RGB_GPIO					GPIOA
#define RGB_PIN						GPIO_PIN_11
#define RGB_TIM						TIM1
#define RGB_TIM_Period 				180					// 144M/0.8Mhz
#define RGB_TIM_Prescaler 			1
#define RGB_Send0 					43					//(int)round(0.24*RGB_TIM_Period)  
#define RGB_Send1 					130					//(int)round(0.72*RGB_TIM_Period)
#define RGB_Reset 					0
#define RGB_ChipsNum				10					//灯带芯片数量
#define RGB_ResetNum				64				//1.25us为信号周期
//限位开关，默认高电平，闭合时低电平
#define MCU_SW_DET					CS1_Econ
#define MCU_SW_DET_CLK				RCC_APB2_PERIPH_GPIOB
#define MCU_SW_DET_GPIO				GPIOB
#define MCU_SW_DET_PIN				GPIO_PIN_4
//报警器，默认低电平，当充电电极短路时高电平
#define MCU_WARM					CS1_Ttig
#define MCU_WARM_CLK				RCC_APB2_PERIPH_GPIOA
#define MCU_WARM_GPIO				GPIOA
#define MCU_WARM_PIN				GPIO_PIN_15
//充电电极短路检测，默认高电平，一切正常才低电平
#define MCU_CH_DET					CS2_Ttig
#define MCU_CH_DET_CLK				RCC_APB2_PERIPH_GPIOB
#define MCU_CH_DET_GPIO				GPIOB
#define MCU_CH_DET_PIN				GPIO_PIN_3
//默认高电平，当红外对齐与限位开关闭合时低电平
#define MCU_CH_DET_ON				CS2_Econ			
#define MCU_CH_DET_ON_CLK			RCC_APB2_PERIPH_GPIOB
#define MCU_CH_DET_ON_GPIO			GPIOB		
#define MCU_CH_DET_ON_PIN			GPIO_PIN_5
void IR_RX_Init(void); 
void NVIC_Config(void);
void IC_Init(void);
uint8_t IrDA_ReceiveData(uint16_t *pdu);
void Key_Init(void);
void Relay_Init(void);
void RGB_Init(void);
void RGB_SetDuty(uint16_t Compare);
void RGB_SetValue(int G,int B,int R);
void RGB_ShowCharging(void);
void RGB_ShowCharged(void);
void RGB_ShowError(void);
void LimitSwitch_Init(void);
void ChargeDetection_Init(void);
void Key_Change_RGB(void);
#endif
