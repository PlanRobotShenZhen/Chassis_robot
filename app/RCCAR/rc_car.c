#include "rc_car.h"


#define TIM1_Period 40000
#define TIM1_Prescaler 72

void RCCAR_Init(void)
{
	/*定义结构体*/
	GPIO_InitType GPIO_InitStructure;
	TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
	OCInitType TIM_OCInitStructure;
	/*开启时钟*/
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);		//36M
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);		//72M
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA |
		RCC_APB2_PERIPH_GPIOB |
		RCC_APB2_PERIPH_AFIO, ENABLE);

	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);		//PA8

	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);		//PB5
	GPIO_ConfigPinRemap(GPIO_PART1_RMP_TIM3, ENABLE);		//TIM3引脚部分重定义
	/*配置时钟源*/
	//TIM_ConfigInternalClk(TIM1);		//选择TIM2为内部时钟，若不调用此函数，TIM默认也为内部时钟

	/*时基单元初始化*/
	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler - 1;	//预分频器，即PSC的值
	TIM_InitTimeBase(TIM1, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM1的时基单元

	TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.Period = TIM1_Period - 1;			//计数周期，即ARR的值
	TIM_TimeBaseInitStructure.Prescaler = TIM1_Prescaler/2 - 1;	//预分频器，即PSC的值
	//TIM_TimeBaseInitStructure.RepetCnt = 0;            			//重复计数器，高级定时器才会用到
	TIM_InitTimeBase(TIM3, &TIM_TimeBaseInitStructure);         //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元


	/*输出比较初始化*/
	TIM_InitOcStruct(&TIM_OCInitStructure);						//结构体初始化，若结构体没有完整赋值
	//则最好执行此函数，给结构体所有成员都赋一个默认值
	//避免结构体初值不确定的问题
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初始的CCR值
	TIM_InitOc1(TIM1, &TIM_OCInitStructure);					//将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
	TIM_ConfigOc1Preload(TIM1, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIM1, ENABLE);							//启用ARR

	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
	TIM_OCInitStructure.Pulse = 2;								//初始的CCR值
	TIM_InitOc2(TIM3, &TIM_OCInitStructure);
	TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIM3, ENABLE);
	/*TIM使能*/
	TIM_Enable(TIM1, ENABLE);
	TIM_Enable(TIM3, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIM3, ENABLE);
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompareTIM1_CH1(uint16_t Compare)
{
	TIM_SetCmp1(TIM1, Compare);		//设置CCR1的值
}

void PWM_SetCompareTIM3_CH2(uint16_t Compare)
{
	TIM_SetCmp2(TIM3, Compare);		//设置CCR2的值
}

void RCCAR_Process(uint16_t ch1, uint16_t ch2)
{
	PWM_SetCompareTIM1_CH1(ch1);
	PWM_SetCompareTIM3_CH2(ch2);
}

