#include "adc.h"

float Voltage = 0.0f,Voltage_All = 0.0f,Voltage_Count = 0.0f;    //��ص�ѹ������صı���  
float g_fltProprity_Voltage = 0.0f;  // ��ص����ı���

int g_nVol_get_Flag = 0;          // �Ƿ��Ѿ������õ���ѹ�ı�־λ

/**************************************************************************
�������ܣ�ACD��ʼ����ص�ѹ���
��ڲ�������
����  ֵ����
**************************************************************************/
void  Adc_Init(void)
{    
 	ADC_InitType ADC_InitStructure; 
	GPIO_InitType GPIO_InitStructure;
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC | RCC_AHB_PERIPH_ADC1, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ConfigAdcHclk(RCC_ADCPLLCLK_DIV2);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	GPIO_InitStructure.Pin = GPIO_PIN_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//��Դ��ѹģ����������
	GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);	
	
	/*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//��λ��ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);*/

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.MultiChEn = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ContinueConvEn = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;	//ת��������������ⲿ��������
	ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;	//ADC�����Ҷ���
	ADC_InitStructure.ChsNumber = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Enable(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_StartCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}		
/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_ConfigRegularChannel(ADC1, ch, 1, ADC_SAMP_TIME_239CYCLES5);	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			     
	ADC_EnableSoftwareStartConv(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ENDC));//�ȴ�ת������
	return ADC_GetDat(ADC1);	//�������һ��ADC1�������ת�����
}

/**************************************************************************
*  �������ܣ��ɼ����ADCֵ��ƽ��ֵ����
*
*  ��ڲ�����ADCͨ���Ͳɼ�����
*
*  �� �� ֵ��ADת�����
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
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ ��λMV
**************************************************************************/

float Get_battery_volt(void)   
{  	
	float fltVol = 0.0f;
	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�
	//fltVol = Get_adc_Average(Battery_Ch, Sample_time) *3.3 * (R16 + R17) / R17 / 4096;	
	fltVol = Get_Adc(Battery_Ch) *3.3 * R16 / R17 / 4096;	
	/*printf("Voltage is %f \r\n", (Get_Adc(Battery_Ch) * 3.3 / 4096));
	printf("Data is %d\r\n",Get_Adc(Battery_Ch));*/
	return fltVol;
}




