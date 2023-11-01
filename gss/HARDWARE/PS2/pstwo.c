#include "pstwo.h"



/**************************************************************************
函数功能：PS2手柄任务
入口参数：无
返回  值：无
**************************************************************************/	
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ));//此任务以200Hz的频率运行 	

    }
}  


/**************************************************************************
函数功能：PS2手柄初始化
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	            //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);					      //根据设定参数初始化GPIOD

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     // inpuT
	GPIO_Init(GPIOE, &GPIO_InitStructure);					      //根据设定参数初始化GPIOE

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOE
}


/**************************************************************************
函数功能：读取PS2手柄的控制量
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Read(void)
{
	      
}


/**************************************************************************
函数功能：向手柄发送命令
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_Cmd(u8 CMD)
{
	
}


/**************************************************************************
函数功能：判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
入口参数：无
返回  值：0：红灯模式，其他：其他模式
**************************************************************************/	
u8 PS2_RedLight(void)
{
	
	return 1;
}


/**************************************************************************
函数功能：读取手柄数据
入口参数：无
返回  值：无
**************************************************************************/	
void PS2_ReadData(void)
{
	
}


//对读出来的PS2的数据进行处理,只处理按键部分  
//只有一个按键按下时按下为0， 未按下为1
u8 PS2_DataKey()
{
	
	return 1;
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	
	return 1;
}


//清除数据缓冲区
void PS2_ClearData()
{
	
}


/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	 
}


//short poll
void PS2_ShortPoll(void)
{
	
}


//进入配置
void PS2_EnterConfing(void)
{
 
	
	
}


//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	
}



//振动设置
void PS2_VibrationMode(void)
{
	
}



//完成并保存配置
void PS2_ExitConfing(void)
{
  
}
//手柄配置初始化
void PS2_SetInit(void)
{

	
}
//读取手柄信息
void PS2_Receive (void)
{
	
}


