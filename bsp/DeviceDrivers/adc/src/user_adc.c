#include "user_adc.h"
#include "rtthread.h"
#include "bsp.h"
#include "n32g45x.h"
#include "485_address.h"
#include "mb.h"
#include "balance.h"

/**
 * @brief  Configures the different GPIO ports.
 */
void ADC_GPIO_Configuration(void)
{
//	int i = 0;
//	GPIO_InitType GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	/* Enable GPIO clock */
//	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC, ENABLE);
//#if(CARMODE == Diff)
//	{
//		uint16_t init_pin[5] = { MCU_MT_CURR_ADC_RxPin ,MCU_ADC_24VARM_IOUT_RxPin ,MCU_ADC_19V_IOUT_RxPin ,MCU_ADC_5V_IOUT_RxPin,MCU_ADC_12VPC_IOUT_RxPin };
//		GPIO_Module* initGpio[5] = { MCU_MT_CURR_ADC_GPIO,MCU_ADC_24VARM_IOUT_GPIO,MCU_ADC_19V_IOUT_GPIO ,MCU_ADC_5V_IOUT_GPIO ,MCU_ADC_12VPC_IOUT_GPIO };
//		for (i = 0;i < 5;i++)
//		{
//			GPIO_InitStructure.Pin = init_pin[i];
//			GPIO_InitPeripheral(initGpio[i], &GPIO_InitStructure);
//		}
//		
//	}
//	
//#else
//	{
//		uint16_t init_pin[7] = { ADC1_JT_RxPin ,ADC2_RxPin ,YL_1_RxPin ,YL_2_RxPin,YL_3_RxPin ,YL_4_RxPin ,YL_5_RxPin };
//		GPIO_Module* initGpio[7] = { ADC1_JT_GPIO,ADC2_GPIO,YL_1_GPIO ,YL_2_GPIO ,YL_3_GPIO ,YL_4_GPIO ,YL_5_GPIO };
//		for (i = 0;i < 7;i++)
//		{
//			if (i == 4)continue;//< YL_3用作自动充电控制输出点
//			GPIO_InitStructure.Pin = init_pin[i];
//			GPIO_InitPeripheral(initGpio[i], &GPIO_InitStructure);
//		}
//	}
//#endif

}

void ADC_Initial(ADC_Module* ADCx)
{
//	ADC_InitType ADC_InitStructure;
//	/* ADC configuration ------------------------------------------------------*/
//	ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;
//	ADC_InitStructure.MultiChEn = DISABLE;
//	ADC_InitStructure.ContinueConvEn = DISABLE;
//	ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
//	ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
//	ADC_InitStructure.ChsNumber = 1;
//	ADC_Init(ADCx, &ADC_InitStructure);


//	/* Enable ADC */
//	ADC_Enable(ADCx, ENABLE);
//	/*Check ADC Ready*/
//	while (ADC_GetFlagStatusNew(ADCx, ADC_FLAG_RDY) == RESET)
//		;
//	/* Start ADC calibration */
//	ADC_StartCalibration(ADCx);
//	/* Check the end of ADC calibration */
//	while (ADC_GetCalibrationStatus(ADCx));
}
/**************************************************************************
函数功能：ACD初始化电池电压检测
入口参数：无
返回  值：无
**************************************************************************/
void  Adc_Init(void)
{
//	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC1| RCC_AHB_PERIPH_ADC2| RCC_AHB_PERIPH_ADC3| RCC_AHB_PERIPH_ADC4, ENABLE);
//	/* RCC_ADCHCLK_DIV16*/
//	ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
//	RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source		
//	ADC_GPIO_Configuration();
//	ADC_Initial(ADC2);
//	ADC_Initial(ADC4);
//#if(CARMODE != Diff)
//	{
//		ADC_Initial(ADC1);
//		ADC_Initial(ADC3);
//	}
//#endif
}

/**************************************************************************
函数功能：配置和获取单通道的模拟信号的ADC（模数转换器）数据
入口参数：无
返回  值：无
**************************************************************************/
uint16_t ADC_GetData(ADC_STRUCT* adc_S)
{
//	ADC_Module* ADCx = adc_S->ADCx;
//	uint8_t ADC_Channel = adc_S->ADC_Channel;
//	// if (adc_S->step == 0){
//	if (!adc_S->step){
//		ADC_ConfigRegularChannel(ADCx, ADC_Channel, 1, ADC_SAMP_TIME_239CYCLES5);
//		/* Start ADC Software Conversion */
//		ADC_EnableSoftwareStartConv(ADCx, ENABLE);
//		adc_S->step = 1;
//	}else if(adc_S->step < 5){
//		if (!ADC_GetFlagStatus(ADCx, ADC_FLAG_ENDC)){
//		// if (ADC_GetFlagStatus(ADCx, ADC_FLAG_ENDC) != 0){
//			ADC_ClearFlag(ADCx, ADC_FLAG_ENDC);
//			ADC_ClearFlag(ADCx, ADC_FLAG_STR);
//			adc_S->value = ADC_GetDat(ADCx);
//			adc_S->step = 0;
//			return 1;
//		}
//		else adc_S->step++;
//	}else{
//		adc_S->step = 0;
//		return 2;//《 出错
//	}
//	return 0;
}


/**************************************************************************
函数功能：ADC任务
入口参数：无
返回  值：无
**************************************************************************/
void ADC_task(void* pvParameters)
{
	
//	int i = 0;

//#if (CARMODE != Diff)
//	static ADC_STRUCT as[7];
//	as[i].ADCx = ADC1_JT_ADC;
//	as[i].ADC_Channel = ADC1_JT_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = ADC2_ADC;
//	as[i].ADC_Channel = ADC2_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = YL_1_ADC;
//	as[i].ADC_Channel = YL_1_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = YL_2_ADC;
//	as[i].ADC_Channel = YL_2_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = YL_3_ADC;
//	as[i].ADC_Channel = YL_3_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = YL_4_ADC;
//	as[i].ADC_Channel = YL_4_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = YL_5_ADC;
//	as[i].ADC_Channel = YL_5_ADC_Channel;
//	as[i].step = 0;
	BatteryInfoInit();
//#else
//	static ADC_STRUCT as[5];
//	as[i].ADCx = MCU_MT_CURR_ADC;
//	as[i].ADC_Channel = MCU_MT_CURR_ADC_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = MCU_ADC_24VARM_IOUT;
//	as[i].ADC_Channel = MCU_ADC_24VARM_IOUT_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = MCU_ADC_19V_IOUT;
//	as[i].ADC_Channel = MCU_ADC_19V_IOUT_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = MCU_ADC_5V_IOUT;
//	as[i].ADC_Channel = MCU_ADC_5V_IOUT_Channel;
//	as[i++].step = 0;
//	as[i].ADCx = MCU_ADC_12VPC_IOUT;
//	as[i].ADC_Channel = MCU_ADC_12VPC_IOUT_Channel;
//	as[i++].step = 0;
//	i = 0;
//	BatteryInfoInit();
//#endif
	while (1)
		{
	rt_thread_delay(100);   //< 10ms
//		
//		uint16_t result = 0;
//		#if (CARMODE != Diff)
//		{
//			static int i = 0;
//			switch (i) 
//			{    // ADC1 轮流转换
//			case 0: result = ADC_GetData(&as[0]);	i++;
//				if (result == 1) pdu[adc1_jt_value] = as[0].value;
//				break;
//			case 1: result = ADC_GetData(&as[2]);	i++;
//				if (result == 1) pdu[adc_yl_1] = as[2].value;
//				break;
//			case 2: result = ADC_GetData(&as[3]);   i++;
//				if (result == 1) pdu[adc_yl_2] = as[3].value;
//				break;
//			case 3: result = ADC_GetData(&as[4]);	i = 0;
//				if (result == 1) pdu[adc_yl_3] = as[4].value; // YL_3 用作自动充电控制输出点
//				break;
//			}
//			if (ADC_GetData(&as[1]) == 1)pdu[adc2_value] = as[1].value;//< ADC2
//			if (ADC_GetData(&as[5]) == 1)pdu[adc_yl_4] = as[5].value;//< ADC3
//			if (ADC_GetData(&as[6]) == 1)pdu[adc_yl_5] = as[6].value;//< ADC4		
			BatteryInformation();
//			SPI1_ReadWriteByte();
			//PowerControl();
	}
//		#else
//		{
//			if (i == 0)//< ADC2 轮流转换
//			{
//				result = ADC_GetData(&as[0]);
//				if (result == 1)
//				{
//					pdu[MT_CURR_Value] = as[0].value;
//					i = 1;
//				}
//				else if (result == 2)
//				{//< err
//					i = 1;
//				}
//			}
//			else if (i == 1)
//			{
//				result = ADC_GetData(&as[1]);
//				if (result == 1)
//				{
//					pdu[IOUT_24VARM_Value] = as[1].value;
//					i = 2;
//				}
//				else if (result == 2)
//				{//< err
//					i = 2;
//				}
//			}
//			else if (i == 2)
//			{
//				result = ADC_GetData(&as[2]);
//				if (result == 1)
//				{
//					pdu[IOUT_19V_Value] = as[2].value;
//					i = 3;
//				}
//				else if (result == 2)
//				{//< err
//					i = 3;
//				}
//			}
//			else if (i == 3)
//			{
//				result = ADC_GetData(&as[3]);
//				if (result == 1)
//				{
//					pdu[IOUT_5V_Value] = as[3].value;
//					i = 0;
//				}
//				else if (result == 2)
//				{//< err
//					i = 0;
//				}
//			}
//			if (ADC_GetData(&as[4]) == 1)pdu[IOUT_12VPC_Value] = as[4].value;//< ADC4
//		}
//		BatteryInformation();
//		#endif
//	}
		
}




