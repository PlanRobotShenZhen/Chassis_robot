#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"


#define Battery_Ch    14       // ADC的通道14
#define Battery_Max   54.0f    // 设置电池的满电状态
#define Sample_time   10       // 设置电池电量的采样次数
#define Potentiometer 8        // 电位器ADC通道8

#define R16           47.0f    //470K
#define R17						2.0f     //20k    分别是2个分压电阻


extern float Voltage, Voltage_All,Voltage_Count;    // 存储读取到的电压值 	
extern float g_fltProprity_Voltage;  // 电池电量的比例

extern int g_nVol_get_Flag;          // 是否已经测量得到电压的标志位

void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);


#endif 


