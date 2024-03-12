#ifndef __RCCAR_H__
#define __RCCAR_H__


extern float tire_speed;//< 轮胎速度，单位公里/小时
extern float mileage;//< 里程计，单位米
extern long EncPos;
void RCCAR_Init(uint16_t* p);
void RCCAR_Process(uint16_t ch1, uint16_t ch2);

#endif
