#include "pstwo.h"



/**************************************************************************
�������ܣ�PS2�ֱ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ));//��������200Hz��Ƶ������ 	

    }
}  


/**************************************************************************
�������ܣ�PS2�ֱ���ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	            //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOD

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     // inpuT
	GPIO_Init(GPIOE, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOE

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOE
}


/**************************************************************************
�������ܣ���ȡPS2�ֱ��Ŀ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Read(void)
{
	      
}


/**************************************************************************
�������ܣ����ֱ���������
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Cmd(u8 CMD)
{
	
}


/**************************************************************************
�������ܣ��ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
��ڲ�������
����  ֵ��0�����ģʽ������������ģʽ
**************************************************************************/	
u8 PS2_RedLight(void)
{
	
	return 1;
}


/**************************************************************************
�������ܣ���ȡ�ֱ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_ReadData(void)
{
	
}


//�Զ�������PS2�����ݽ��д���,ֻ����������  
//ֻ��һ����������ʱ����Ϊ0�� δ����Ϊ1
u8 PS2_DataKey()
{
	
	return 1;
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	
	return 1;
}


//������ݻ�����
void PS2_ClearData()
{
	
}


/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	 
}


//short poll
void PS2_ShortPoll(void)
{
	
}


//��������
void PS2_EnterConfing(void)
{
 
	
	
}


//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	
}



//������
void PS2_VibrationMode(void)
{
	
}



//��ɲ���������
void PS2_ExitConfing(void)
{
  
}
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{

	
}
//��ȡ�ֱ���Ϣ
void PS2_Receive (void)
{
	
}


