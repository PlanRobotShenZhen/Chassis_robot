#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"


void KEY_Init(void);
unsigned char click(void);
void Delay_ms(void);
unsigned char click_N_Double (u8 time);
unsigned char click_N_Double_MPU6050 (u8 time);
unsigned char Long_Press(void);


/*--------KEY control pin--------*/

#define KEY_PORT	GPIOD
#define KEY_PIN		GPIO_Pin_8
#define KEY			PDin(8) 
/*----------------------------------*/
#endif 
