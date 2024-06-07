#ifndef __REMOTE__h
#define __REMOTE__h

#include "delay.h"
#include "system.h"
#include "bsp.h"

extern bool Sbus_Data_Parsing_Flag;	//��ģ���ݴ����־λ

void Remote_Task(void *pvParameters);
void Sbus_Data_Parsing(void);
void MotorEnableFunction(void);
#endif
