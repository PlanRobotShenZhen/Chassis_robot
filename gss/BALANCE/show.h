#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "system.h"

/*------------ �����ջ��С -------------*/
#define SHOW_TASK_PRIO		3
#define SHOW_STK_SIZE 		512  

extern int Voltage_Show;          //��ѹ��ʾȫ�ֱ���


void show_task(void *pvParameters);


#endif
