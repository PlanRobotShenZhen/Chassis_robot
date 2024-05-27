#include "show.h"
#include "user_can.h"
#include "485_address.h"
int Voltage_Show = 0;          //电压显示全局变量

/**************************************************************************
函数功能：LED的显示功能、电机心跳计时功能、电压显示功能
入口参数：无
返回  值：无
**************************************************************************/
void show_task(void *pvParameters)
{
  u32 lastWakeTime = getSysTickCnt();
	static int nFlag = 0;                //启动标志位，上电后10s才会执行后面的控制任务程序
																
	while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//此任务以50Hz的频率运行
			
			g_nHeart_Time_LB++;
			g_nHeart_Time_LF++;
			g_nHeart_Time_RF++;
			g_nHeart_Time_RB++;
			
			Voltage_All += Get_battery_volt();     //多次采集电压数据累积
			if(++Voltage_Count == 20)
			{
				// 采集20次电池电压数据,多次采集求平均值
				Voltage = Voltage_All / 20;
				(getPDUData())[power_voltage] = Voltage*100;
				Voltage_All = 0;
				Voltage_Count = 0;
				// 计算出电池电压的占比
				g_fltProprity_Voltage = Voltage / Battery_Max;
				g_nVol_get_Flag = 1;             // 进行控制的标志位
				
			}
			
			if(Deviation_Count >= CONTROL_DELAY)
			{
				if(nFlag == 0)
				{
					nFlag++;
					// 根据读取到的陀螺仪的状态来使能机器人的
					// Flag_Stop = 0;     //0代表使能机器人
				}
			}
    }
}  


