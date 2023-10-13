#include "can.h"
#include "user_can.h"
#include "system.h"
#include "485_address.h"

float Wheel_perimeter = 0;    //�����ܳ�����λ���ף�
float Wheel_spacing = 0;      //�������־� ����λ���ף�//�������robot_select_init.h�ļ��г�ʼ��
float Axle_spacing = 0;       //ǰ�����
float Omni_turn_radiaus = 0;  //ȫ����ת��뾶

Remote_Control_struct *rc_ptr;
Motor_struct *motorA_ptr;
Motor_struct *motorB_ptr;
Motor_struct *motorC_ptr;
Motor_struct *motorD_ptr;
//Robot_speed *robot_speed_ptr;
enum CarMode g_emCarMode = UNKNOW;                                                //С������
unsigned char Flag_Stop = 0;                                                      //1����ʹ�ܻ�����
enum ENUM_CarControl_Mode g_eControl_Mode = CONTROL_MODE_UNKNOW;  // �����˵Ŀ��Ʒ�ʽ
unsigned char g_ucRemote_Flag = 0;              //��ģ������־λ
unsigned char g_ucRos_Flag = 0;                 // ROS��λ�������־λ 

struct Motor_parameter  MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;

/****************************
* �������ܣ�����ʱ��Դʹ���ⲿ����ʱ��HSE
* unDiv��RCC_PLLSource_HSE_Div1/RCC_PLLSource_HSE_Div2/RCC_PLLSource_HSI_Div2
*        �˲����Ĺ��������ʱ��Դ����Դ��
* unPllm�����õ��Ǳ�Ƶϵ����RCC_PLLMul_2 ~ RCC_PLLMul_16
* 
****************************/
void RCC_HSE_Config(unsigned int unDiv, unsigned int unPllm)
{
	 RCC_DeInit();//���ⲿRCC�Ĵ�������Ϊȱʡֵ
	 RCC_HSEConfig(RCC_HSE_ON); //�����ⲿ���پ���(HSE)
	 if(RCC_WaitForHSEStartUp() == SUCCESS) //�ȴ�HSE����
	 {
			RCC_HCLKConfig(RCC_SYSCLK_Div1); //����AHB(HCLK)ʱ��
			RCC_PCLK1Config(RCC_HCLK_Div2); //����APB1����ʱ��(PCLK1)
			RCC_PCLK2Config(RCC_HCLK_Div1); //����APB2����ʱ��(PCLK2)
		 //����PLLʱ��Դ(u32Div)�Լ��䱶Ƶϵ��(u32Pill),
		 //�����ⲿ����ʱ�ӵļ���Ƶ,�ڶ�������ΪPLL���ʱ�Ӽ���Ƶ,
		 //Ĭ����9,��Ϊ72MHz?
			RCC_PLLConfig(unDiv,unPllm);
			RCC_PLLCmd(ENABLE);//ʹ�ܻ���ʧ��PLL
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //���ָ����RCC��־λ�������, PLL����
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //����ϵͳʱ��(SYSCLK)
			while(RCC_GetSYSCLKSource()!=0x08);//��������ϵͳʱ�ӵ�ʱ��Դ,0x08:PLL��Ϊϵͳʱ��
	 }
}



// �ֶ������λ
void Soft_Reset()
{
	int nDelay = 0;
	__DSB();
	while(nDelay < 100)
	{
		nDelay++;
	}
}
	

void systemInit(void)
{
	int nTime = 0;
	int i = turn_off_remote;
	uint16_t *pdu = getPDUData();
	while(nTime < 2000)
	{
		nTime++;
	}	
	// ����ϵͳʱ��Ϊ72MHz��ʹ���ⲿ���پ���HSE
	RCC_HSE_Config(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	
	delay_init();					          //��ʼ����ʱ����
	LED_Init();                     //��ʼ����LED���ӵ�Ӳ���ӿ�
	USART1_Init(1500000);	        //=====���ڳ�ʼ��Ϊ����ͨ�Ĵ��ڣ���ӡ������Ϣ DMA
	Usart3_init(115200);            //����λ��ͨ�ų�ʼ��������3
	Usart5_Init(100000);            //����5��ʼ�������ں�ģ����
	//Adc_Init();                     //�ɼ���ص�ѹADC���ų�ʼ��	
	Can_Driver_Init();              //�ײ�canЭ���ʼ��
	
	Robot_Select();                 // ���ݵ�λ����ֵ�ж�Ŀǰ�������е�����һ������ˣ�
	                                // Ȼ����ж�Ӧ�Ĳ�����ʼ��
	modbus_task_init();
	pdu[car_type] = 4;
	pdu[car_version] = 0x88;
	//��ʼ����ģ����
	rc_ptr = (Remote_Control_struct*) &pdu[i];

	pdu[i++] = TURN_OFF_REMOTE;
	pdu[i++] = TURN_ON_REMOTE;
	pdu[i++] = VEL_BASE_VALUE;
	pdu[i++] = DIR_BASE_VALUE;
	pdu[i++] = LIMIT_MAX_VAL;
	pdu[i++] = LIMIT_MIN_VAL;
	pdu[i++] = SPEED_LEVEL1;
	pdu[i++] = SPEED_LEVEL2;
	pdu[i++] = SPEED_LEVEL3;
	pdu[i++] = SPEED_LOW;
	pdu[i++] = SPEED_MIDDLE;
	pdu[i++] = SPEED_HIGH;
	pdu[i++] = SPEED_DIR_LOW;
	pdu[i++] = SPEED_DIR_MIDDLE;
	pdu[i++] = SPEED_DIR_HIGH;
	pdu[i++] = LIGHT_BASE;
	pdu[i++] = LIGHT_MAX;
	pdu[i++] = LIGHT_MIN;
	//��ʼ���������
	i = motor1_state;
	int length = motor2_state-motor1_state;
	motorA_ptr = (Motor_struct*)&pdu[i];
	motorB_ptr = (Motor_struct*)&pdu[i+length];
	motorC_ptr = (Motor_struct*)&pdu[i+2*length];
	motorD_ptr = (Motor_struct*)&pdu[i+3*length];
	
	for( int j = 0; j < 4; j++){
		pdu[(i++)+j*length] = 0;   //�ڵ�״̬
		pdu[(i++)+j*length] = FourWheer_Radiaus*10000; //���ְ뾶������10000��
		pdu[(i++)+j*length] = REDUCTION_RATE*100;  //���ٱȣ�����100��
		pdu[(i++)+j*length] = j+1;   //CAN ID
		pdu[(i++)+j*length] = 1000;  //CAN�����ʣ�����100��
		pdu[(i++)+j*length] = 0;  //����
		pdu[(i++)+j*length] = 0;  //����
		pdu[(i++)+j*length] = 0;  //Ŀ��ת��
		pdu[(i++)+j*length] = 0;
		pdu[(i++)+j*length] = 0;  //Ŀ��ת��
		pdu[(i++)+j*length] = 0;
		pdu[(i++)+j*length] = 0; //Ŀ��λ��
		pdu[i+j*length] = 0;
	}
	//��ʼ��С���ٶ��޷�����
	i = car_max_lin_speed;
	union {
		float v;
		int16_t ud[2];
	}tmp;
	tmp.v = 0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = 0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i] = 0;
	

	
}
