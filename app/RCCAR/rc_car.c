#include "rc_car.h"


#define TIM1_Period 40000
#define TIM1_Prescaler 72

void RCCAR_Init(void)
{
	/*����ṹ��*/
	GPIO_InitType GPIO_InitStructure;
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	OCInitType TIM_OCInitStructure;
	/*����ʱ��*/
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);		//36M
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);		//72M
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA |
		RCC_APB2_PERIPH_GPIOB |
		RCC_APB2_PERIPH_AFIO, ENABLE);

	/*GPIO��ʼ��*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);		//PA8

	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);		//PB5
	GPIO_ConfigPinRemap(GPIO_PART1_RMP_TIM3, ENABLE);		//TIM3���Ų����ض���
	/*����ʱ��Դ*/
	//TIM_ConfigInternalClk(TIM1);		//ѡ��TIM2Ϊ�ڲ�ʱ�ӣ��������ô˺�����TIMĬ��ҲΪ�ڲ�ʱ��

	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//�������ڣ���ARR��ֵ
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler - 1;	//Ԥ��Ƶ������PSC��ֵ
	TIM_InitTimeBase(TIM1, &TIM_TimeBaseInitStructure);         //���ṹ���������TIM_TimeBaseInit������TIM1��ʱ����Ԫ

	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//�������ڣ���ARR��ֵ
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler/2 - 1;	//Ԥ��Ƶ������PSC��ֵ
	//TIM_TimeBaseInitStructure.RepetCnt = 0;            			//�ظ����������߼���ʱ���Ż��õ�
	TIM_InitTimeBase(TIM3, &TIM_TimeBaseInitStructure);         //���ṹ���������TIM_TimeBaseInit������TIM3��ʱ����Ԫ


	/*����Ƚϳ�ʼ��*/
	TIM_InitOcStruct(&TIM_OCInitStructure);						//�ṹ���ʼ�������ṹ��û��������ֵ
	//�����ִ�д˺��������ṹ�����г�Ա����һ��Ĭ��ֵ
	//����ṹ���ֵ��ȷ��������
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//����Ƚ�ģʽ��ѡ��PWMģʽ1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//���ʹ��
	TIM_OCInitStructure.Pulse = 2;								//��ʼ��CCRֵ
	TIM_InitOc1(TIM1, &TIM_OCInitStructure);					//���ṹ���������TIM_OC1Init������TIM1������Ƚ�ͨ��1
	TIM_ConfigOc1Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIM1, ENABLE);							//����ARR

	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//����Ƚ�ģʽ��ѡ��PWMģʽ1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//���ʹ��
	TIM_OCInitStructure.Pulse = 2;								//��ʼ��CCRֵ
	TIM_InitOc2(TIM3, &TIM_OCInitStructure);
	TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIM3, ENABLE);
	/*TIMʹ��*/
	TIM_Enable(TIM1, ENABLE);
	TIM_Enable(TIM3, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIM3, ENABLE);
}

/**
  * ��    ����PWM����CCR
  * ��    ����Compare Ҫд���CCR��ֵ����Χ��0~100
  * �� �� ֵ����
  * ע�����CCR��ARR��ͬ����ռ�ձȣ��˺���������CCR��ֵ������ֱ����ռ�ձ�
  *           ռ�ձ�Duty = CCR / (ARR + 1)
  */
void PWM_SetCompareTIM1_CH1(uint16_t Compare)
{
	TIM_SetCmp1(TIM1, Compare);		//����CCR1��ֵ
}

void PWM_SetCompareTIM3_CH2(uint16_t Compare)
{
	TIM_SetCmp2(TIM3, Compare);		//����CCR2��ֵ
}

void RCCAR_Process(uint16_t ch1, uint16_t ch2)
{
	PWM_SetCompareTIM1_CH1(ch1);
	PWM_SetCompareTIM3_CH2(ch2);
}

