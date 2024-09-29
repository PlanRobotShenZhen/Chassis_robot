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
* �������ܣ�ϵͳ��ʼ������
* ��    ������
* �� �� ֵ����
**************************************************/
void systemInit(void)
{
	rt_thread_delay(200);   //< 20ms
	Usart1_Init(GetUsart1_baud(pdu[moddbus_485_baud]));	  //=====���ڳ�ʼ��Ϊ����ͨ�Ĵ��ڣ���ӡ������Ϣ DMA
	Usart3_Init(115200);            //����λ��ͨ�ų�ʼ��������3
#if (CARMODE != Diff)
	{
		Uart4_Init(9600);              //����4��ʼ�������ڶ�ȡ�����Ϣ
		ExioInit();
	}
#endif	
    Uart5_Init(100000);            //����5��ʼ�������ں�ģ����
	Can_Driver_Init(pdu[CAN_baud]);              //�ײ�canЭ���ʼ��
    
	GPIO_Init();                     //��ʼ����GPIO���ӵ�Ӳ���ӿ�
	rt_thread_delay(27000);   			//< 2700ms
	Motor_Init(pdu[motor_number]);
	InitMotorParameters(); 
	DiyFunctionBasedCar(pdu[car_type]);
	
}

