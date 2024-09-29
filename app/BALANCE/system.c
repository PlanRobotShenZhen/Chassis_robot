#include "can.h"
#include "user_can.h"
#include "system.h"
#include "JDQ_EN.h"
#include "mb.h"
#include "usartx.h"
#include "485_address.h"
#include "robot_select_init.h"
#include "led.h"
#include "balance.h"
#include "motor.h"
#include "user_adc.h"
#include "FLASH_WR.h"
#include "CircularCar.h"
ROBOT_CONTROL robot_control = {0};
/**************************************************
* 函数功能：系统初始化函数
* 参    数：无
* 返 回 值：无
**************************************************/
void systemInit(void)
{
	rt_thread_delay(200);   //< 20ms
	Usart1_Init(GetUsart1_baud(pdu[moddbus_485_baud]));	  //=====串口初始化为，普通的串口，打印调试信息 DMA
	Usart3_Init(115200);            //上下位机通信初始化，串口3
#if (CARMODE != Diff)
	{
		Uart4_Init(9600);              //串口4初始化，用于读取电池信息
		ExioInit();
	}
#endif	
    Uart5_Init(100000);            //串口5初始化，用于航模控制
	Can_Driver_Init(pdu[CAN_baud]);              //底层can协议初始化
    
	GPIO_Init();                     //初始化与GPIO连接的硬件接口
	rt_thread_delay(27000);   			//< 2700ms
	Motor_Init(pdu[motor_number]);
	InitMotorParameters(); 
	DiyFunctionBasedCar(pdu[car_type]);
	
}

