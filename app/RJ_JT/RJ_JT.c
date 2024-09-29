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
        rt_thread_delay(20);//��������2ms��Ƶ������
        Soft_JT_Flag = SWD_JT_Control();
    }
}

/**************************************************************************
�������ܣ�����SWD���������ͣ
��ڲ�������
����  ֵ����
**************************************************************************/
bool SWD_JT_Control(void)
{
    //�������ɵ�Դ��exio_input.bit.X0�͵�ƽʱ��ͣ�������Ѳ���
    if((Abs_int(pdu[rc_ch9_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) || (GPIO_ReadInputDataBit(ESTOP_SW_IN_GPIO, ESTOP_SW_IN_PIN) == RESET))
    {
        //GPIO_ResetBits(RJ_JT_GPIO,RJ_JT_Pin);
        return true;
    }
    else
    {
        //GPIO_SetBits(RJ_JT_GPIO,RJ_JT_Pin);
        return false;
    }
}


//#endif

