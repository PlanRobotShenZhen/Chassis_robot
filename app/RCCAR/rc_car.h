#ifndef __RCCAR_H__
#define __RCCAR_H__


extern float tire_speed;//< ��̥�ٶȣ���λ����/Сʱ
extern float mileage;//< ��̼ƣ���λ��
extern long EncPos;
void RCCAR_Init(uint16_t* p);
void RCCAR_Process(uint16_t ch1, uint16_t ch2);

#endif
