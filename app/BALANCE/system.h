#include "stdbool.h"
#include "stdint.h"

#ifndef __SYSTEM_H
#define __SYSTEM_H



/*--------------- 初始化时的相关标志位-----------------------*/
//extern enum CarMode g_emCarMode;          //机器人选型标志位
void Soft_Reset(void);  // 手动软件复位

void systemInit(void);
uint32_t GetUsart1_baud(uint16_t data);
void DiyFunctionBasedCar(uint16_t data);









#endif /* __SYSTEM_H */
