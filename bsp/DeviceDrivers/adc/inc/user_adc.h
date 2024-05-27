#ifndef __ADC_H
#define __ADC_H	

#include "n32g45x.h"

typedef struct __ADC_STRUCT__
{
	ADC_Module* ADCx;
	uint8_t ADC_Channel;
	uint8_t step;
	uint16_t value;
}ADC_STRUCT;

void Adc_Init(void);
void ADC_task(void* pvParameters);
void BatteryInfoInit(void);

#endif 


