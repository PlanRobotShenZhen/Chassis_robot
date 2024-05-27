#include "n32g45x.h"
#include "rc_car.h"
#include "485_address.h"
#include "mb.h"
#include "balance.h"

#define TIM1_Period 40000
#define TIM1_Prescaler 72
static int16_t enc_old = 0;
static int16_t diff_enc = 0;
long EncPos = 0;

float ratio=60;//< 减速比
float encoder_accuracy=4;//< 编码器精�?
float tire_diameter=66;//< �?胎直�?
float tire_speed = 0;//< �?胎速度，单位公�?/小时
float mileage = 0;//< 里程计，单位�?

float tire_speed_factor=1;//< 速度计算因数
float mileage_factor;

TIM_Module* TIML = TIM1;
TIM_Module* TIMA = TIM2;
TIM_Module* TIMx = TIM3;

/**************************************************************************
�������ܣ����ת�� r/min ת��Ϊ���ת�� �������ٶ� m/s
��ڲ�����fltSpeed��������ת�٣�m/s
�� �� ֵ���������ٶ� nReturn = MotorRotation / 60 / REDUCTION_RATE * C;
**************************************************************************/
void EncodeTimeInit(void)
{
	TIM_TimeBaseInitType TIM_TimeBaseStructure;
	TIM_ICInitType TIM_ICInitStructure;
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);		
	//initial tim for encode
	TIM_DeInit(TIMx);
	TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.Period = 0xffff;
	TIM_TimeBaseStructure.Prescaler = 0;
	TIM_TimeBaseStructure.ClkDiv = TIM_CLK_DIV1;
	TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
	TIM_InitTimeBase(TIMx, &TIM_TimeBaseStructure);
	//qr encode set
	TIM_InitIcStruct(&TIM_ICInitStructure);
	TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_RISING;
	TIM_ICInitStructure.IcFilter = 0x02;
	TIM_ICInitStructure.Channel = TIM_CH_1;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ICInitStructure.Channel = TIM_CH_2;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);
	TIM_ConfigEncoderInterface(TIMx, TIM_ENCODE_MODE_TI12, TIM_IC_POLARITY_BOTHEDGE, TIM_IC_POLARITY_BOTHEDGE);

	TIM_ConfigArPreload(TIMx, ENABLE);
	TIM_SetCnt(TIMx, 0);
	TIM_Enable(TIMx, ENABLE);

	//TIM_ClearFlag(TIMx, TIM_FLAG_UPDATE);
	//TIM_ConfigInt(TIMx, TIM_INT_UPDATE, ENABLE);
}

/**************************************************************************
�������ܣ����ת�� r/min ת��Ϊ���ת�� �������ٶ� m/s
��ڲ�����fltSpeed��������ת�٣�m/s
�� �� ֵ���������ٶ� nReturn = MotorRotation / 60 / REDUCTION_RATE * C;
**************************************************************************/
void RCCAR_Init(void)
{
	/*定义结构�?*/
	GPIO_InitType GPIO_InitStructure;
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	OCInitType TIM_OCInitStructure;
	/*开�?时钟*/
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

	/*GPIO初�?�化*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = GPIO_PIN_8;//PA8 RJ_JT 前进
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_3;//PB3 CS2_Ttig 左右
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_ALL_RMP_TIM2, ENABLE);
	/*配置时钟�?*/
	//TIM_ConfigInternalClk(TIM1);		//选择TIM2为内部时钟，若不调用此函数，TIM默�?�也为内部时�?

	/*时基单元初�?�化*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分�?�，选择不分频，此参数用于配�?滤波器时钟，不影响时基单元功�?
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的�?
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler - 1;	//预分频器，即PSC的�?
	TIM_InitTimeBase(TIML, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单�?

	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分�?�，选择不分频，此参数用于配�?滤波器时钟，不影响时基单元功�?
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的�?
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler/2 - 1;	//预分频器，即PSC的�?
	//TIM_TimeBaseInitStructure.RepetCnt = 0;            			//重�?��?�数�?，高级定时器才会用到
	TIM_InitTimeBase(TIMA, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单�?


	/*输出比较初�?�化*/
	TIM_InitOcStruct(&TIM_OCInitStructure);						//结构体初始化，若结构体没有完整赋�?
	//则最好执行�?�函数，给结构体所有成员都赋一�?默�?��?
	//避免结构体初值不�?定的�?�?
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取�?
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初�?�的CCR�?
	TIM_InitOc1(TIML, &TIM_OCInitStructure);					//将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
	TIM_ConfigOc1Preload(TIML, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIML, ENABLE);							//�?用ARR

	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取�?
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初�?�的CCR�?
	TIM_InitOc2(TIMA, &TIM_OCInitStructure);
	TIM_ConfigOc2Preload(TIMA, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIMA, ENABLE);
	/*TIM使能*/
	TIM_Enable(TIML, ENABLE);
	TIM_Enable(TIMA, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIML, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIMA, ENABLE);


	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.Pin = GPIO_PIN_4| GPIO_PIN_5; //< CS1_Econ CS2_Econ
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_PART1_RMP_TIM3, ENABLE);
	EncodeTimeInit();

	ratio = pdu[rc_ratio]*0.01;
	if (ratio == 0)ratio = 120;
	encoder_accuracy = pdu[rc_encoder_accuracy];
	if (encoder_accuracy == 0)encoder_accuracy = 4;
	tire_diameter = pdu[rc_tire_diameter];
	if (tire_diameter == 0)tire_diameter = 66;

	tire_speed_factor = 3.6f * (3.14f * tire_diameter);
	tire_speed_factor  /= (encoder_accuracy * ratio);
	if (tire_speed_factor == 0)tire_speed_factor = 1;
	
	mileage_factor = 3.14f * tire_diameter * 0.01f;
	mileage_factor /= encoder_accuracy * ratio;
	if (mileage_factor == 0)mileage_factor = 1;
}

/**************************************************************************
�������ܣ�����TIML��ͨ��1�ıȽ�ֵ
��ڲ�����Compare - �Ƚ�ֵ
�� �� ֵ����
**************************************************************************/
void PWM_SetCompareTIML_CH1(uint16_t Compare)
{
	TIM_SetCmp1(TIML, Compare);		// ����TIML��CCR1ΪCompare
}


/**************************************************************************
�������ܣ�����TIMA��ͨ��2�ıȽ�ֵ
��ڲ�����fCompare - �Ƚ�ֵ
�� �� ֵ����
**************************************************************************/
void PWM_SetCompareTIMA_CH2(uint16_t Compare)
{
	TIM_SetCmp2(TIMA, Compare);		// ����TIMA��CCR2ΪCompare
}

/**************************************************************************
�������ܣ�RC��������������������PWM�Ƚ�ֵ����ȡ��������������̺ͳ��ٵ�
��ڲ�����ch1 - PWMͨ��1�ıȽ�ֵ��ch2 - PWMͨ��2�ıȽ�ֵ
�� �� ֵ����
**************************************************************************/
void RCCAR_Process(uint16_t ch1, uint16_t ch2)
{
	int16_t enc;// ������ֵ
	PWM_SetCompareTIML_CH1(ch1);// ����PWMͨ��1��ͨ��2�ıȽ�ֵ
	PWM_SetCompareTIMA_CH2(ch2);
	enc = TIMx->CNT;// ��ȡ��������ֵ
	diff_enc = enc - enc_old;// ����������仯��
	if (pdu[rc_encoder_dir] == 1){
		diff_enc = myabs(diff_enc);
	}
	enc_old = enc;
	EncPos += diff_enc;
	pdu[rc_encoder_high] = EncPos >> 16;//���������
	pdu[rc_encoder_low] = EncPos;
	// ������̥�ٶȺ����
	tire_speed = diff_enc * tire_speed_factor;
	mileage = EncPos * mileage_factor;
	pdu[rc_car_speed] = (int16_t)((int)tire_speed);
	pdu[rc_car_mileage_high] = ((int)mileage) >> 16;
	pdu[rc_car_mileage_low] = (int)mileage;

	if (pdu[rc_encoder_reset] == 5){
		pdu[rc_encoder_reset] = 0;
		TIMx->CNT = 0;
		EncPos = 0;
	}
}

