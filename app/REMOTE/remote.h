#ifndef __REMOTE__h
#define __REMOTE__h

#include "delay.h"
#include "system.h"
#include "bsp.h"

extern bool Sbus_Data_Parsing_Flag;	//航模数据处理标志位

void Remote_Task(void *pvParameters);
void Sbus_Data_Parsing(void);
void MotorEnableFunction(void);
#endif
