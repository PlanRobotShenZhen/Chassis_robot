#include "Charger.h"
#include <rtthread.h>
#include "led.h"
#include  "485_address.h"
/*�������ݲ��ֲ���*/
int Receive_i = 3; 		//��������ָ�룬�Ӹ�λ��ʼ����
uint8_t guide_flag = 0;	//���յ�����λ
uint16_t high_count = 0;//���ռƴ�
int BitEnd = 0;			//���յ�һ��������

/*RGB����*/
uint8_t ChipEndFlag = 0;
uint8_t RGB_ChangeFlag = 0;
uint8_t RGB_ArrayNum = 0;
int RGB_i = 23;//RGBָ��
int bit_i = 7;//��λ���ݵ�λָ��
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
/*������ն˳�ʼ��*/
void IR_RX_Init(void)
{
	GPIO_InitType GPIO_InitStructure;	
	RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);
	//MCU_INF_RX��ʼ��
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
/*�������벶���ʼ��*/
void IC_Init(void)
{
	
	/*����ʱ��*/
	RCC_EnableAPB1PeriphClk(MCU_INF_RX_TIM_CLK, ENABLE);
	RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);
	
	/*GPIO��ʼ��*/
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_INF_RX_GPIO,&GPIO_InitStructure);
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1 ;    //ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; //������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.Period = 65536 - 1;        //�������ڣ���ARR��ֵ��IC��������󣬷�ֹ�������
	TIM_TimeBaseInitStructure.Prescaler = 7200 - 1;      //72M/7200=10^4KHz����0.1ms����һ��
	TIM_TimeBaseInitStructure.RepetCnt = 0;            	 
	TIM_InitTimeBase(MCU_INF_RX_TIM, &TIM_TimeBaseInitStructure);
	/*PWMIģʽ��ʼ��*/
	TIM_ICInitType TIM_ICInitStructure;
	TIM_ICInitStructure.Channel = TIM_CH_1;						//ѡ�����ö�ʱ��ͨ��1
	TIM_ICInitStructure.IcFilter = 0xF;							//�����˲�������0x0~0xF�����Թ����źŶ���
	TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_FALLING;	//���ԣ�ѡ��Ϊ�½��ش�������
	TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;			//����Ԥ��Ƶ��ѡ�񲻷�Ƶ��n���źŴ���һ�β���
	TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;//�����źŽ��棬ѡ��ֱͨ��������
	TIM_ConfigPwmIc(MCU_INF_RX_TIM, &TIM_ICInitStructure);		//����TIM5�����벶��ͨ��
																//�˺���ͬʱ�����һ��ͨ������Ϊ�෴�����ã�ʵ��PWMIģʽ
																//���˲�����Ƶһ�£�����Ϊ�½��ش������źŽ��棻
	/*�ж��������*/
	TIM_ClearFlag(MCU_INF_RX_TIM,TIM_FLAG_CC2);
	TIM_ConfigInt(MCU_INF_RX_TIM,TIM_INT_CC2,ENABLE);
	/*�½��ش���ʱ����cnt*/
	TIM_SelectInputTrig(MCU_INF_RX_TIM,TIM_TRIG_SEL_TI1FP1);
	TIM_SelectSlaveMode(MCU_INF_RX_TIM, TIM_SLAVE_MODE_RESET);
	/*TIMʹ��*/
	TIM_Enable(MCU_INF_RX_TIM, ENABLE);			
}
/*�����������*/
uint8_t IrDA_ReceiveData(uint16_t *pdu)
{	
	static uint8_t ReceiveData = 0x0;
	static uint8_t temp;								//��temp�������֣����帳ֵ��RD��
	static uint16_t NoSignalCout = 0;
	if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO,MCU_INF_RX_PIN) == SET)
	{
		 if (NoSignalCout >=10000)
		{
			//��ֹ���
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
		//��������û���յ��źţ��򷵻س���쳣�ź�
		pdu[BatteryTemperature] = 3;
		return 3;
	}
	if (BitEnd == 1)
	{
		high_count = TIM_GetCap2(MCU_INF_RX_TIM);
		if (high_count>=900 && high_count <= 1100)		//���յ�����λ
			{
				guide_flag = 1;	
				temp = ReceiveData;
			}	
		else if (guide_flag == 1)
		{
			if (high_count>=500 && high_count < 900)	//���յ�1
			{
				//�ߵ�ƽ70ms
				//����iλ�������:�����㣬��1��1����0����				
				temp  |= (1 << Receive_i);	
				
			} 
			else if (high_count>=100 && high_count < 500)//���յ�0
			{
				//�ߵ�ƽ30ms
				//����iλ��������������㣬��1���䣬��0��0
				temp  &= ~(1 << Receive_i);		
			}
			Receive_i --;								//����ָ������
			if (Receive_i < 0)							//������4λ����
			{
				Receive_i = 3;
				guide_flag = 0;
				ReceiveData = temp;						//ÿ������λˢ��һ������
			}
		}
	}
	BitEnd = 0;
	//pdu[BatteryTemperature] = 3;
	return ReceiveData;
}
/*��������ж�*/
void TIM5_IRQHandler(void)
{
    if (TIM_GetIntStatus(MCU_INF_RX_TIM, TIM_INT_CC2) == SET)
    {
		//�����ش������պ�����������־
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
	/*�̵�����ʼ��*/
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
	/*����ʱ��*/
	RCC_EnableAPB2PeriphClk(RGB_CLK, ENABLE);
	RCC_EnableAPB2PeriphClk(RGB_TIM_CLK, ENABLE);
	/*GPIO��ʼ��*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = RGB_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(RGB_GPIO,&GPIO_InitStructure);
	
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.Period = RGB_TIM_Period -1;		//�������ڣ���ARR��ֵ
	TIM_TimeBaseInitStructure.Prescaler = RGB_TIM_Prescaler -1;	//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInitStructure.RepetCnt = 1;
	TIM_InitTimeBase(RGB_TIM, &TIM_TimeBaseInitStructure);     	//���ṹ���������TIM_TimeBaseInit������TIM3��ʱ����Ԫ
	/*����Ƚϳ�ʼ��*/
	TIM_InitOcStruct(&TIM_OCInitStructure);	
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//����Ƚ�ģʽ��ѡ��PWMģʽ1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//���ʹ��
	TIM_OCInitStructure.Pulse = 0;								//��ʼ��CCRֵ

	TIM_InitOc4(RGB_TIM, &TIM_OCInitStructure);					//���ṹ���������TIM_OC1Init������TIM1������Ƚ�ͨ��1
	TIM_ConfigOc4Preload(RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);		//����TIMԤ���ع��ܣ�CCR������ʱ����Ӱ�쵱ǰPWM���¸�����ʱ��ʱ��Ч��
	TIM_ConfigArPreload(RGB_TIM, ENABLE);						//����ARR	
	/*�ж��������*/
	TIM_ClearFlag(RGB_TIM,TIM_INT_UPDATE);
	TIM_ConfigInt(RGB_TIM,TIM_INT_UPDATE,ENABLE);
	/*TIMʹ��*/
	TIM_Enable(RGB_TIM, ENABLE);
	TIM_EnableCtrlPwmOutputs(RGB_TIM, ENABLE);
	//RGB_SetDuty(RGB_Send1);
}
void RGB_SetDuty(uint16_t Compare)
{
	TIM_SetCmp4(RGB_TIM, Compare);		//����CCR1��ֵ
}
/**************************************************************************
�������ܣ��趨�ƹ��RGBֵ
��ڲ�������ԭɫ�Ƶ�RGBֵ�������ж�ʱ��ԭ��ÿ������λ���ݸı�һ�ε�λ��
������Ϣ��PA11(YL_7)Ϊ�ź������
**************************************************************************/
void RGB_SetValue(int G,int B,int R)
{

	int bit = 2;
	//RGBȡλ����
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
			//RGB���ݴ�����ɱ�־λ
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
//�̵Ƽ��һ����˸
void RGB_ShowCharging(void)
{
	static int delay_i = 0;
	
	delay_i++;
	//ÿһ���л�һ����ɫ
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
//�̵Ƴ���
void RGB_ShowCharged(void)
{
	static int delay_i = 0;

	delay_i++;
	//ÿһ���л�һ����ɫ
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
//��Ƽ��һ����˸
void RGB_ShowError(void)
{
	static int delay_i = 0;
	delay_i++;
	//ÿһ���л�һ����ɫ
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
�������ܣ����ж��иı䶨ʱ��ռ�ձȣ���ʵ��RGBֵ�ĸı�
��ڲ������ޣ������ж�ʱ��ԭ����ÿ��λ�����еĸ�λΪֵ��ÿ������λ�����жϸı�ռ�ձȣ�
������Ϣ��PA8(RJ_JT)Ϊ�������룬ÿ�ΰ����л�һ��RGBֵ
**************************************************************************/
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetIntStatus(RGB_TIM, TIM_INT_UPDATE) == SET)
	{
		static int Reset_i = RGB_ResetNum;
		static int RGB_chips = RGB_ChipsNum;
		TIM_ClrIntPendingBit(RGB_TIM, TIM_INT_UPDATE);			
		//���յ���ɫ�ź�
		if (RGB_ChangeFlag == 1)
		{
			//����reset��
			if (Reset_i > 0)
			{
				RGB_SetDuty(RGB_Reset);
				Reset_i--;
				Reset_i--;
			}
			//��η���ÿ��оƬ��rgb����
			else if (RGB_chips > 0)
			{

				RGB_SetValue(RGB_Array[RGB_ArrayNum][0], RGB_Array[RGB_ArrayNum][1], RGB_Array[RGB_ArrayNum][2]);					
				if(ChipEndFlag == 1)
				{
					ChipEndFlag = 0;
					RGB_chips--;
				}

			}
			//һ����RGB���ݷ������
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
	//Ĭ�ϸߵ�ƽ��TRAVEL_SW���ź�ʱ�͵�ƽ
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(MCU_SW_DET_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_SW_DET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_SW_DET_GPIO, &GPIO_InitStructure);
}
void ChargeDetection_Init(void)
{
	//Ĭ�ϸߵ�ƽ�����缫��·�����͵�ƽʱ�缫����
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(MCU_CH_DET_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.Pin = MCU_CH_DET_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_CH_DET_GPIO, &GPIO_InitStructure);
	//Ĭ�ϸߵ�ƽ���͵�ƽʱС������׮�Խӳɹ�
	RCC_EnableAPB2PeriphClk(MCU_CH_DET_ON_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.Pin = MCU_CH_DET_ON_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(MCU_CH_DET_ON_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);

}
