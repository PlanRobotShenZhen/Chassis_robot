#include "stdbool.h"
#include "stdint.h"

#ifndef __SYSTEM_H
#define __SYSTEM_H



/*--------------- ��ʼ��ʱ����ر�־λ-----------------------*/
//extern enum CarMode g_emCarMode;          //������ѡ�ͱ�־λ
void Soft_Reset(void);  // �ֶ������λ

void systemInit(void);
uint32_t GetUsart1_baud(uint16_t data);
void DiyFunctionBasedCar(uint16_t data);









#endif /* __SYSTEM_H */
