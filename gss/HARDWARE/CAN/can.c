#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "usartx.h"
#include "user_can.h"
#include "motor_data.h"


int g_nMotorLF_config = -1;
int g_nMotorLB_config = -1;  
int g_nMotorRF_config = -1;
int g_nMotorRB_config = -1;

union URcv_Vel_Data uVelLF;
union URcv_Vel_Data uVelLB;
union URcv_Vel_Data uVelRF;
union URcv_Vel_Data uVelRB;  //�������ʵʱ�ٶȵı���

union URcv_ERROR_Data uError; // �����������Ĺ���״̬


int g_nReInitMotor_LB = 0;   //�����������ϵ��������δ�ϵ磬���³�ʼ��������������
int g_nReInitMotor_LF = 0;   //Ϊ1������Ҫ���³�ʼ����Ϊ0�������Ҫ���³�ʼ��
int g_nReInitMotor_RF = 0;
int g_nReInitMotor_RB = 0;


int g_nHeart_count_LB = 0;    //���������������ڹ涨������û��������������Ϊ�ӻ�����
int g_nHeart_count_LF = 0;    //����״̬����ΪSTATE_STOP
int g_nHeart_count_RF = 0;
int g_nHeart_count_RB = 0;

int g_nHeart_Lastcount_LB = 0;  //��¼�ϴν����������ļ���ֵ
int g_nHeart_Lastcount_LF = 0;
int g_nHeart_Lastcount_RF = 0;
int g_nHeart_Lastcount_RB = 0;




enum ENUM_HEART_STAT eLF_Motor_Heart = STATE_UNKNOW;
enum ENUM_HEART_STAT eLB_Motor_Heart = STATE_UNKNOW;
enum ENUM_HEART_STAT eRF_Motor_Heart = STATE_UNKNOW; 
enum ENUM_HEART_STAT eRB_Motor_Heart = STATE_UNKNOW;

enum ENUM_ERROR_STATE eLF_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eLB_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eRF_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eRB_Motor_Error = ERROR_NONE;   // �������������ȡ

/**********************************************************
* �������ܣ� stm32�ײ��canͨ��Э���ʼ����
* ������     ��
* ˵����     ��
**********************************************************/
void Can_Driver_Init(void)
{

	GPIO_InitType GPIO_InitStructure;
	CAN_InitType CAN_InitStructure;
	CAN_FilterInitType CAN_FilterInitStructure;

#if CAN_RX0_INT_ENABLE
	NVIC_InitType NVIC_InitStructure;
#endif

	//��Ӧ��GPIO�ں�ʱ�ӳ�ʼ��
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1, ENABLE); //ʹ��CANʱ��
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE); //ʹ��AFIOʱ��

	//��Ҫ��CAN���ܸ��ã�ӳ�䵽PD0��PD1��
	GPIO_ConfigPinRemap(GPIO_RMP2_CAN1, ENABLE);

	//��ʼ���õ���GPIO��
	GPIO_InitStructure.Pin = GPIO_PIN_0;	  // PD0��ΪRx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������ģʽ
	GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_1;		// PD1��ΪTx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������,ʹ�ø���CAN����
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO���ٶ�Ϊ50MHz
	GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);

	//����ģʽ�������ʳ�ʼ��
	// CAN��Ԫ����
	CAN_InitStructure.TTCM = DISABLE;		  //��ʱ�����ͨ��ģʽ
	CAN_InitStructure.ABOM = DISABLE;		  //����Զ����߹���
	CAN_InitStructure.AWKUM = DISABLE;		  //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.NART = ENABLE;		  //��ֹ�����Զ��ش�
	CAN_InitStructure.RFLM = DISABLE;		  //���Ĳ�����,�µĸ��Ǿɵ�
	CAN_InitStructure.TXFP = DISABLE;		  //���ȼ��ɱ��ı�ʶ������
	CAN_InitStructure.OperatingMode = CAN_Operating_NormalMode; //ģʽ����
	CAN_InitStructure.RSJW = CAN_RSJW_1tq;	  //
	CAN_InitStructure.TBS1 = CAN_TBS1_3tq;
	CAN_InitStructure.TBS2 = CAN_TBS2_2tq;
	CAN_InitStructure.BaudRatePrescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);

	//����CANɸѡ��
	//���ù�����
	CAN_FilterInitStructure.Filter_Num = 0;					 //������0
	CAN_FilterInitStructure.Filter_Mode = CAN_Filter_IdMaskMode;	 //�����ڱ�ʶ������ģʽ��
	CAN_FilterInitStructure.Filter_Scale = CAN_Filter_32bitScale; // 32λ
	CAN_FilterInitStructure.Filter_HighId = 0x0000;				 ////32λID
	CAN_FilterInitStructure.Filter_LowId = 0x0000;
	CAN_FilterInitStructure.FilterMask_HighId = 0x0000; // 32λMASK
	CAN_FilterInitStructure.FilterMask_LowId = 0x0000;
	CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_Filter_FIFO0; //������0������FIFO0
	CAN_FilterInitStructure.Filter_Act = ENABLE;				 //ʹ�ܹ�����0
	CAN1_InitFilter(&CAN_FilterInitStructure);							 //��������ʼ��

	//����CAN�ж�
#if CAN_RX0_INT_ENABLE

	CAN_INTConfig(CAN1, CAN_INT_FMP0, ENABLE); // FIFO0��Ϣ�Һ��ж�����.	���յ����ݾͻ�����ж�
	// CAN_ClearFlag(CAN1,CAN_IT_FMP0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
}




/**********************************************************
* �������ܣ� can�жϷ�����	
* ������     ��
* ˵����     ���жϺ����л�ȡ��ʵʱ�������ٶ�
**********************************************************/
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMessage RxMessage;
	
	CAN_ReceiveMessage(CAN1, 0, &RxMessage);
	
	// ���PDO����
	if (RxMessage.StdId >= 0x181 && RxMessage.StdId <= 0X1FF)
	{//< ���PDO����
		int id = RxMessage.StdId - 0x181;
		int i = 0;
		MOTOR_TPDO* md;
		if (id < 10)
		{
			for (i = 0;i < MAX_MOTOR_NUMBER;i++)
			{
				if (id == mrd[i].d.mapping)
				{
					id = i+1;// �ҵ�ӳ����ID
					break;
				}
			}
			i = 0;
			mrd[id].d.online = 1;
			md = &mtd[id];
			md->d.heartbeat = 0;
			md->d.status.sd = RxMessage.Data[i++];
			md->d.status.sd |= RxMessage.Data[i++] << 8;
			md->d.current_velocity = RxMessage.Data[i++];
			md->d.current_velocity |= RxMessage.Data[i++] << 8;
			md->d.current_velocity |= RxMessage.Data[i++] << 16;
			md->d.current_velocity |= RxMessage.Data[i++] << 24;
		}
	}
	else
	{//< SDO ����

	}

	
	
	CAN_ClearFlag(CAN1,CAN_INT_FMP0);
	
}
#endif

