#include "show.h"
#include "user_can.h"
#include "485_address.h"
int Voltage_Show = 0;          //��ѹ��ʾȫ�ֱ���

/**************************************************************************
�������ܣ�LED����ʾ���ܡ����������ʱ���ܡ���ѹ��ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void show_task(void *pvParameters)
{
  u32 lastWakeTime = getSysTickCnt();
	static int nFlag = 0;                //������־λ���ϵ��10s�Ż�ִ�к���Ŀ����������
																
	while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//��������50Hz��Ƶ������
			
			g_nHeart_Time_LB++;
			g_nHeart_Time_LF++;
			g_nHeart_Time_RF++;
			g_nHeart_Time_RB++;
			
			Voltage_All += Get_battery_volt();     //��βɼ���ѹ�����ۻ�
			if(++Voltage_Count == 20)
			{
				// �ɼ�20�ε�ص�ѹ����,��βɼ���ƽ��ֵ
				Voltage = Voltage_All / 20;
				(getPDUData())[power_voltage] = Voltage*100;
				Voltage_All = 0;
				Voltage_Count = 0;
				// �������ص�ѹ��ռ��
				g_fltProprity_Voltage = Voltage / Battery_Max;
				g_nVol_get_Flag = 1;             // ���п��Ƶı�־λ
				
			}
			
			if(Deviation_Count >= CONTROL_DELAY)
			{
				if(nFlag == 0)
				{
					nFlag++;
					// ���ݶ�ȡ���������ǵ�״̬��ʹ�ܻ����˵�
					// Flag_Stop = 0;     //0����ʹ�ܻ�����
				}
			}
    }
}  


