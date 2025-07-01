#include "RJ_JT.h"
#include "bsp.h"
#include "rtthread.h"
#include "485_address.h"
#include "mb.h"
#include "usartx.h"
#include "balance.h"
//#if(CARMODE != Diff)

bool Soft_JT_Flag = false;

/**************************************************************************
�������ܣ�SWD�����ͣ����
��ڲ�������
����  ֵ����
**************************************************************************/
void RJJT_task(void *pvParameters)
{
    while(1)
    {
        rt_thread_delay(200);//��������2ms��Ƶ������
		Soft_JT_Flag = SWD_JT_Control();
    }
}

/**************************************************************************
�������ܣ�����SWD���������ͣ
��ڲ�������
����  ֵ����
**************************************************************************/
bool soft_emergency_stop = false;
bool SWD_JT_Control(void)
{
    if(GPIO_ReadInputDataBit(ESTOP_SW_IN_GPIO, ESTOP_SW_IN_PIN) == SET) {
        // ������, �򿪼�ͣ
        return true;
    } else if (Abs_int(pdu[EmergencyStop_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR){
        // ABS ��ͣ����
        soft_emergency_stop = true;
    } else if ((Abs_int(pdu[Torque_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)){
        // ABS ȡ����ͣ
        soft_emergency_stop = false;
    } else if(ROS_RecvFlag) {
        // ��λ����ͣ
        if(Receive_Data[2] == 1)
        {
            soft_emergency_stop = true;
        }
        else if(Receive_Data[2] == 2)
        {
            soft_emergency_stop = false;
        }
    }
    return soft_emergency_stop;
}


