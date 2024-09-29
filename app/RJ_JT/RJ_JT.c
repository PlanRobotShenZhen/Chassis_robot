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
        rt_thread_delay(20);//此任务以2ms的频率运行
        Soft_JT_Flag = SWD_JT_Control();
    }
}

/**************************************************************************
函数功能：根据SWD控制软件急停
入口参数：无
返回  值：无
**************************************************************************/
bool SWD_JT_Control(void)
{
    //阿克曼旧电源板exio_input.bit.X0低电平时急停，基本已不用
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

