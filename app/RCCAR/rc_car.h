#ifndef __RCCAR_H__
#define __RCCAR_H__

#include "stdint.h"


extern float tire_speed;//< ��̥�ٶȣ���λ����/Сʱ
extern float mileage;//< ��̼ƣ���λ��
extern long EncPos;
void RCCAR_Init(void);
void RCCAR_Process(uint16_t ch1, uint16_t ch2);

#endif
