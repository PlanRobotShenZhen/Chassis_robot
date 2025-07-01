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
函数功能：SWD软件急停功能
入口参数：无
返回  值：无
**************************************************************************/
void RJJT_task(void *pvParameters)
{
    while(1)
    {
        rt_thread_delay(200);//此任务以2ms的频率运行
		Soft_JT_Flag = SWD_JT_Control();
    }
}

/**************************************************************************
函数功能：根据SWD控制软件急停
入口参数：无
返回  值：无
**************************************************************************/
bool soft_emergency_stop = false;
bool SWD_JT_Control(void)
{
    if(GPIO_ReadInputDataBit(ESTOP_SW_IN_GPIO, ESTOP_SW_IN_PIN) == SET) {
        // 物理开关, 打开急停
        return true;
    } else if (Abs_int(pdu[EmergencyStop_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR){
        // ABS 急停开关
        soft_emergency_stop = true;
    } else if ((Abs_int(pdu[Torque_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)){
        // ABS 取消急停
        soft_emergency_stop = false;
    } else if(ROS_RecvFlag) {
        // 上位机急停
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


