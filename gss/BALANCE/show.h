#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "system.h"

/*------------ 任务堆栈大小 -------------*/
#define SHOW_TASK_PRIO		3
#define SHOW_STK_SIZE 		512  

extern int Voltage_Show;          //电压显示全局变量


void show_task(void *pvParameters);


#endif
