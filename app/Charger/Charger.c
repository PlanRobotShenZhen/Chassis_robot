#include "Charger.h"
#include <rtthread.h>
#include "led.h"
#include  "485_address.h"
/*接收数据部分参数*/
int Receive_i = 3; 		//接收数据指针，从高位开始发送
uint8_t guide_flag = 0;	//接收到引导位
uint16_t high_count = 0;//接收计次
int BitEnd = 0;			//接收到一个完整字

/*RGB参数*/
uint8_t ChipEndFlag = 0;
uint8_t RGB_ChangeFlag = 0;
uint8_t RGB_ArrayNum = 0;
int RGB_i = 23;//RGB指针
int bit_i = 7;//八位数据的位指针
//int RGB_Array[6][3];
int RGB_Array[][3] = {
	//{0xFF,0x00,0x00},
	{0xFF,0x7F,0x00},
	{0xFF,0xFF,0x00},
	{0x00,0xFF,0x00},
	{0x00,0xFF,0xFF},
	{0x00,0x00,0xFF},
	{0x8B,0x00,0xFF}
};
int RGB_Charging[][3] = {
	{0x00,0x00,0x00},
	{0x00,0xFF,0x00},
	{0x00,0x00,0x00},
	{0x00,0xFF,0x00},
	{0x00,0x00,0x00},
	{0x00,0xFF,0x00}
};
int RGB_Charged[][3] = {
	{0x00,0xFF,0x00},
	{0x00,0xFF,0x00},
	{0x00,0xFF,0x00},
	{0x00,0xFF,0x00},
	{0x00,0xFF,0x00},
	{0x00,0xFF,0x00}
};
int RGB_Error[][3] = {
	{0xFF,0x00,0x00},
	{0x00,0x00,0x00},
	{0xFF,0x00,0x00},
	{0x00,0x00,0x00},
	{0xFF,0x00,0x00},
	{0x00,0x00,0x00},
};
/*红外接收端初始化*/
void IR_RX_Init(void)
{
	GPIO_InitType GPIO_InitStructure;	
	RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);
	//MCU_INF_RX初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_INF_RX_GPIO,&GPIO_InitStructure);
}

void NVIC_Config(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitType NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel = MCU_INF_RX_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;		
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = RGB_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;	
	NVIC_Init(&NVIC_InitStructure);
}
/*红外输入捕获初始化*/
void IC_Init(void)
{
	
	/*开启时钟*/
	RCC_EnableAPB1PeriphClk(MCU_INF_RX_TIM_CLK, ENABLE);
	RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_INF_RX_GPIO,&GPIO_InitStructure);
	/*时基单元初始化*/
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1 ;    //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = 65536 - 1;        //计数周期，即ARR的值，IC中设置最大，防止计数溢出
	TIM_TimeBaseInitStructure.Prescaler = 7200 - 1;      //72M/7200=10^4KHz，即0.1ms计数一次
	TIM_TimeBaseInitStructure.RepetCnt = 0;            	 
	TIM_InitTimeBase(MCU_INF_RX_TIM, &TIM_TimeBaseInitStructure);
	/*PWMI模式初始化*/
	TIM_ICInitType TIM_ICInitStructure;
	TIM_ICInitStructure.Channel = TIM_CH_1;						//选择配置定时器通道1
	TIM_ICInitStructure.IcFilter = 0xF;							//输入滤波器参数0x0~0xF，可以过滤信号抖动
	TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_FALLING;	//极性，选择为下降沿触发捕获
	TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;			//捕获预分频，选择不分频，n次信号触发一次捕获
	TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;//输入信号交叉，选择直通，不交叉
	TIM_ConfigPwmIc(MCU_INF_RX_TIM, &TIM_ICInitStructure);		//配置TIM5的输入捕获通道
																//此函数同时会把另一个通道配置为相反的配置，实现PWMI模式
																//即滤波、分频一致；极性为下降沿触发；信号交叉；
	/*中断输出配置*/
	TIM_ClearFlag(MCU_INF_RX_TIM,TIM_FLAG_CC2);
	TIM_ConfigInt(MCU_INF_RX_TIM,TIM_INT_CC2,ENABLE);
	/*下降沿触发时置零cnt*/
	TIM_SelectInputTrig(MCU_INF_RX_TIM,TIM_TRIG_SEL_TI1FP1);
	TIM_SelectSlaveMode(MCU_INF_RX_TIM, TIM_SLAVE_MODE_RESET);
	/*TIM使能*/
	TIM_Enable(MCU_INF_RX_TIM, ENABLE);			
}
/*红外接收数据*/
uint8_t IrDA_ReceiveData(uint16_t *pdu)
{	
	static uint8_t ReceiveData = 0x0;
	static uint8_t temp;								//在temp操作单字，整体赋值给RD。
	static uint16_t NoSignalCout = 0;
	if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO,MCU_INF_RX_PIN) == SET)
	{
		 if (NoSignalCout >=10000)
		{
			//防止溢出
			NoSignalCout = 200;
		}
		NoSignalCout ++;
	}
	else 
	{
		NoSignalCout = 0;
	}
	if (NoSignalCout >= 200)
	{
		//连续两秒没接收到信号，则返回充电异常信号
		pdu[BatteryTemperature] = 3;
		return 3;
	}
	if (BitEnd == 1)
	{
		high_count = TIM_GetCap2(MCU_INF_RX_TIM);
		if (high_count>=900 && high_count <= 1100)		//接收到引导位
			{
				guide_flag = 1;	
				temp = ReceiveData;
			}	
		else if (guide_flag == 1)
		{
			if (high_count>=500 && high_count < 900)	//接收到1
			{
				//高电平70ms
				//左移i位填入变量:或运算，或1置1，或0不变				
				temp  |= (1 << Receive_i);	
				
			} 
			else if (high_count>=100 && high_count < 500)//接收到0
			{
				//高电平30ms
				//左移i位填入变量：与运算，与1不变，与0置0
				temp  &= ~(1 << Receive_i);		
			}
			Receive_i --;								//接收指针右移
			if (Receive_i < 0)							//接收完4位数据
			{
				Receive_i = 3;
				guide_flag = 0;
				ReceiveData = temp;						//每接收四位刷新一次数据
			}
		}
	}
	BitEnd = 0;
	//pdu[BatteryTemperature] = 3;
	return ReceiveData;
}
/*红外接收中断*/
void TIM5_IRQHandler(void)
{
    if (TIM_GetIntStatus(MCU_INF_RX_TIM, TIM_INT_CC2) == SET)
    {
		//上升沿触发接收函数的启动标志
		BitEnd = 1;
		TIM_ClrIntPendingBit(MCU_INF_RX_TIM, TIM_INT_CC2);
    }	
}

void Key_Init(void)
{
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(KEY_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.Pin = KEY_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(KEY_GPIO, &GPIO_InitStructure);
	GPIO_ResetBits(KEY_GPIO, KEY_PIN);

}

void Key_Change_RGB(void)
{
	if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET)
	{
		while (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET);
		RGB_ChangeFlag = 1;
		RGB_ArrayNum++;
		if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
		{
			RGB_ArrayNum = 0;
		}
	}
}

void Relay_Init(void)
{
	/*继电器初始化*/
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(MCU_RELAY1_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.Pin = MCU_RELAY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_RELAY1_GPIO,&GPIO_InitStructure);
	GPIO_ResetBits(MCU_RELAY1_GPIO,MCU_RELAY1_PIN);

}

void RGB_Init(void)
{
	GPIO_InitType GPIO_InitStructure;
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;        	
	OCInitType TIM_OCInitStructure;
	/*开启时钟*/
	RCC_EnableAPB2PeriphClk(RGB_CLK, ENABLE);
	RCC_EnableAPB2PeriphClk(RGB_TIM_CLK, ENABLE);
	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = RGB_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(RGB_GPIO,&GPIO_InitStructure);
	
	
	/*时基单元初始化*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = RGB_TIM_Period -1;		//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.Prescaler = RGB_TIM_Prescaler -1;	//预分频器，即PSC的值
	TIM_TimeBaseInitStructure.RepetCnt = 1;
	TIM_InitTimeBase(RGB_TIM, &TIM_TimeBaseInitStructure);     	//将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
	/*输出比较初始化*/
	TIM_InitOcStruct(&TIM_OCInitStructure);	
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 0;								//初始的CCR值

	TIM_InitOc4(RGB_TIM, &TIM_OCInitStructure);					//将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
	TIM_ConfigOc4Preload(RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);		//启用TIM预加载功能，CCR被加载时不会影响当前PWM，下个更新时间时生效。
	TIM_ConfigArPreload(RGB_TIM, ENABLE);						//启用ARR	
	/*中断输出配置*/
	TIM_ClearFlag(RGB_TIM,TIM_INT_UPDATE);
	TIM_ConfigInt(RGB_TIM,TIM_INT_UPDATE,ENABLE);
	/*TIM使能*/
	TIM_Enable(RGB_TIM, ENABLE);
	TIM_EnableCtrlPwmOutputs(RGB_TIM, ENABLE);
	//RGB_SetDuty(RGB_Send1);
}
void RGB_SetDuty(uint16_t Compare)
{
	TIM_SetCmp4(RGB_TIM, Compare);		//设置CCR1的值
}
/**************************************************************************
函数功能：设定灯光的RGB值
入口参数：三原色灯的RGB值（由于中断时长原因，每传送两位数据改变一次电位）
引脚信息：PA11(YL_7)为信号输出线
**************************************************************************/
void RGB_SetValue(int G,int B,int R)
{

	int bit = 2;
	//RGB取位数据
	if (RGB_i >= 16 && RGB_i < 24)
	{
		bit = (R >> bit_i) & 0x01;
	}
	else if (RGB_i >= 8 && RGB_i < 16) 
	{
		bit = (G >> bit_i) & 0x01;
	}
	else if (RGB_i >= 0 && RGB_i < 8)
	{
		bit = (B >> bit_i) & 0x01;
		if (RGB_i == 1)
		{
			//RGB数据传输完成标志位
			ChipEndFlag = 1;
		}
	}
	bit_i--;
	RGB_i--;
	bit_i--;
	RGB_i--;
	if (bit_i < 0)
	{
		bit_i = 7;
		if (ChipEndFlag == 1)
		{
			RGB_i = 23;
		}
	}

	if (bit == 1)
	{
		RGB_SetDuty(RGB_Send1);
	}
	else if (bit == 0)
	{
		RGB_SetDuty(RGB_Send0);
	}
}
//绿灯间隔一秒闪烁
void RGB_ShowCharging(void)
{
	static int delay_i = 0;
	
	delay_i++;
	//每一秒切换一次颜色
	if (delay_i >= 100)
	{
		delay_i = 0;
		RGB_ChangeFlag = 1;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 3; j++) {
				RGB_Array[i][j] = RGB_Charging[i][j];
			}
		}
		RGB_ArrayNum++;
		if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
		{
			RGB_ArrayNum = 0;
		}
	}
	
}
//绿灯常亮
void RGB_ShowCharged(void)
{
	static int delay_i = 0;

	delay_i++;
	//每一秒切换一次颜色
	if (delay_i >= 100)
	{
		delay_i = 0;
		RGB_ChangeFlag = 1;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 3; j++) {
				RGB_Array[i][j] = RGB_Charged[i][j];
			}
		}
		RGB_ArrayNum++;
		if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
		{
			RGB_ArrayNum = 0;
		}
	}

}
//红灯间隔一秒闪烁
void RGB_ShowError(void)
{
	static int delay_i = 0;
	delay_i++;
	//每一秒切换一次颜色
	if (delay_i >= 100)
	{
		delay_i = 0;
		RGB_ChangeFlag = 1;
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 3; j++) {
				RGB_Array[i][j] = RGB_Error[i][j];
			}
		}
		RGB_ArrayNum++;
		if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
		{
			RGB_ArrayNum = 0;
		}
	}

}
/**************************************************************************
函数功能：在中断中改变定时器占空比，以实现RGB值的改变
入口参数：无（由于中断时长原因，以每两位数据中的高位为值，每发送两位进入中断改变占空比）
引脚信息：PA8(RJ_JT)为按键输入，每次按键切换一次RGB值
**************************************************************************/
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetIntStatus(RGB_TIM, TIM_INT_UPDATE) == SET)
	{
		static int Reset_i = RGB_ResetNum;
		static int RGB_chips = RGB_ChipsNum;
		TIM_ClrIntPendingBit(RGB_TIM, TIM_INT_UPDATE);			
		//接收到变色信号
		if (RGB_ChangeFlag == 1)
		{
			//发送reset码
			if (Reset_i > 0)
			{
				RGB_SetDuty(RGB_Reset);
				Reset_i--;
				Reset_i--;
			}
			//逐次发送每颗芯片的rgb数据
			else if (RGB_chips > 0)
			{

				RGB_SetValue(RGB_Array[RGB_ArrayNum][0], RGB_Array[RGB_ArrayNum][1], RGB_Array[RGB_ArrayNum][2]);					
				if(ChipEndFlag == 1)
				{
					ChipEndFlag = 0;
					RGB_chips--;
				}

			}
			//一整套RGB数据发送完毕
			else if (RGB_chips == 0)
			{
				RGB_SetDuty(RGB_Reset);
				RGB_ChangeFlag = 0;
				Reset_i = RGB_ResetNum;
				RGB_chips = RGB_ChipsNum;
								
			}
		}
	}
}

void LimitSwitch_Init(void)
{
	//默认高电平，TRAVEL_SW有信号时低电平
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(MCU_SW_DET_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_SW_DET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_SW_DET_GPIO, &GPIO_InitStructure);
}
void ChargeDetection_Init(void)
{
	//默认高电平（充电电极短路），低电平时电极正常
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(MCU_CH_DET_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_CH_DET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_CH_DET_GPIO, &GPIO_InitStructure);
	//默认高电平，低电平时小车与充电桩对接成功
	RCC_EnableAPB2PeriphClk(MCU_CH_DET_ON_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.Pin = MCU_CH_DET_ON_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_CH_DET_ON_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);

}
