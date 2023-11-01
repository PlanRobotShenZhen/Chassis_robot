#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"
//LED 端口定义

#define LED_TASK_PRIO		3     //任务优先级
#define LED_STK_SIZE 		128   //任务堆栈大小


/*--------Buzzer control pin--------*/

#define LED_PORT GPIOE
#define LED_PIN GPIO_PIN_10
#define LED PEout(10) 
/*----------------------------------*/

#define LED_PORT     GPIOE
#define LED_R        GPIO_PIN_2
#define LED_G        GPIO_PIN_3
#define LED_B        GPIO_PIN_4
#define LED_Battery  GPIO_PIN_5
#define JDQ_PORT     GPIOD   
#define JDQ_PIN11    GPIO_PIN_11
#define JDQ_PIN12    GPIO_PIN_12
#define JDQ_PORT_RCC  RCC_APB2Periph_GPIOD

void LED_Init(void);  
void led_task(void *pvParameters);
void Led_LightRGB(void);
void Led_LightCar(void);
extern int Led_Count;
#endif
