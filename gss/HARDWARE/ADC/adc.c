#include "adc.h"

float Voltage = 0.0f,Voltage_All = 0.0f,Voltage_Count = 0.0f;    //电池电压采样相关的变量  
float g_fltProprity_Voltage = 0.0f;  // 电池电量的比例

int g_nVol_get_Flag = 0;          // 是否已经测量得到电压的标志位

/**************************************************************************
函数功能：ACD初始化电池电压检测
入口参数：无
返回  值：无
**************************************************************************/
void  Adc_Init(void)
{    
 	ADC_InitType ADC_InitStructure; 
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC | RCC_AHB_PERIPH_ADC1, ENABLE );	  //使能ADC1通道时钟
	RCC_ConfigAdcHclk(RCC_ADCPLLCLK_DIV2);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	GPIO_InitStructure.Pin = GPIO_PIN_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//电源电压模拟输入引脚
	GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);	
	
	/*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//电位器模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);*/

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.MultiChEn = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ContinueConvEn = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;	//转换由软件而不是外部触发启动
	ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;	//ADC数据右对齐
	ADC_InitStructure.ChsNumber = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
	ADC_Enable(ADC1, ENABLE);	//使能指定的ADC1
	ADC_StartCalibration(ADC1);	//使能复位校准  	 
	while(ADC_GetCalibrationStatus(ADC1));	//等待复位校准结束	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
}		
/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_ConfigRegularChannel(ADC1, ch, 1, ADC_SAMP_TIME_239CYCLES5);	//ADC1,ADC通道,采样时间为239.5周期	  			     
	ADC_EnableSoftwareStartConv(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ENDC));//等待转换结束
	return ADC_GetDat(ADC1);	//返回最近一次ADC1规则组的转换结果
}

/**************************************************************************
*  函数功能：采集多次ADC值求平均值函数
*
*  入口参数：ADC通道和采集次数
*
*  返 回 值：AD转换结果
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val += Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压 单位MV
**************************************************************************/

float Get_battery_volt(void)   
{  	
	float fltVol = 0.0f;
	//电阻分压，具体根据原理图简单分析可以得到
	//fltVol = Get_adc_Average(Battery_Ch, Sample_time) *3.3 * (R16 + R17) / R17 / 4096;	
	fltVol = Get_Adc(Battery_Ch) *3.3 * R16 / R17 / 4096;	
	/*printf("Voltage is %f \r\n", (Get_Adc(Battery_Ch) * 3.3 / 4096));
	printf("Data is %d\r\n",Get_Adc(Battery_Ch));*/
	return fltVol;
}




