#include "rc_car.h"
#include "485_address.h"


#define TIM1_Period 40000
#define TIM1_Prescaler 72
uint16_t* pdu;
static int16_t enc_old = 0;
static int16_t diff_enc = 0;
long EncPos = 0;
TIM_Module* TIML = TIM1;//< 线速度
TIM_Module* TIMA = TIM2;//< 角速度
TIM_Module* TIMx = TIM3;
//Use Tim3
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
void RCCAR_Init(uint16_t* p)
{
	/*定义结构体*/
	GPIO_InitType GPIO_InitStructure;
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	OCInitType TIM_OCInitStructure;
	pdu = p;
	/*开启时钟*/
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA |
		RCC_APB2_PERIPH_GPIOB |
		RCC_APB2_PERIPH_AFIO, ENABLE);

	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = GPIO_PIN_8;//PA8
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_3;//PB3
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_ALL_RMP_TIM2, ENABLE);
	/*配置时钟源*/
	//TIM_ConfigInternalClk(TIM1);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟

	/*时基单元初始化*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler - 1;	//预分频器，即PSC的值
	TIM_InitTimeBase(TIML, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元

	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler/2 - 1;	//预分频器，即PSC的值
	//TIM_TimeBaseInitStructure.RepetCnt = 0;            			//重复计数器，高级定时器才会用到
	TIM_InitTimeBase(TIMA, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元


	/*输出比较初始化*/
	TIM_InitOcStruct(&TIM_OCInitStructure);						//结构体初始化，若结构体没有完整赋值
	//则最好执行此函数，给结构体所有成员都赋一个默认值
	//避免结构体初值不确定的问题
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初始的CCR值
	TIM_InitOc1(TIML, &TIM_OCInitStructure);					//将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
	TIM_ConfigOc1Preload(TIML, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIML, ENABLE);							//启用ARR

	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初始的CCR值
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
	GPIO_InitStructure.Pin = GPIO_PIN_4| GPIO_PIN_5;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_PART1_RMP_TIM3, ENABLE);
	EncodeTimeInit();
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompareTIML_CH1(uint16_t Compare)
{
	TIM_SetCmp1(TIML, Compare);		//设置CCR1的值
}

void PWM_SetCompareTIMA_CH2(uint16_t Compare)
{
	TIM_SetCmp2(TIMA, Compare);		//设置CCR2的值
}

void RCCAR_Process(uint16_t ch1, uint16_t ch2)
{
	int16_t enc = TIMx->CNT;
	PWM_SetCompareTIML_CH1(ch1);
	PWM_SetCompareTIMA_CH2(ch2);
	diff_enc = enc - enc_old;
	if (pdu[rc_encoder_dir] == 1)diff_enc = abs(diff_enc);
	enc_old = enc;
	EncPos += diff_enc;
	pdu[rc_encoder_high] = EncPos >> 16;
	pdu[rc_encoder_low] = EncPos;
	if (pdu[rc_encoder_reset]==5)
	{
		pdu[rc_encoder_reset] = 0;
		TIMx->CNT = 0;
		EncPos = 0;
	}
}

