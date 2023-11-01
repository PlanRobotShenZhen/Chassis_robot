#include "led.h"
#include "adc.h"
#include "usartx.h"

int Led_Count;//LED闪烁控制
/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
//void LED_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能端口时钟
//	
//	GPIO_InitStructure.GPIO_Pin = LED_R | LED_G | LED_B | LED_Battery | LED_PIN;	//端口配置
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
//	GPIO_Init(LED_PORT, &GPIO_InitStructure);			//根据设定参数初始化GPIO
//	GPIO_ResetBits(LED_PORT, LED_Battery);             // 初始化的时候，将每一个IO口的电平设置为低
//	GPIO_SetBits(LED_PORT, LED_R | LED_G | LED_B);
//	GPIO_SetBits(LED_PORT,LED_PIN);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
// 
// JDQ_ENStructure1.GPIO_Pin=JDQ_PIN11;  
// JDQ_ENStructure1.GPIO_Mode=GPIO_Mode_Out_PP;  
// JDQ_ENStructure1.GPIO_Speed=GPIO_Speed_50MHz;  
// GPIO_Init(JDQ_PORT,&JDQ_ENStructure1);     
// 

// JDQ_ENStructure2.GPIO_Pin=JDQ_PIN12; 
// JDQ_ENStructure2.GPIO_Mode=GPIO_Mode_Out_PP;  
// JDQ_ENStructure2.GPIO_Speed=GPIO_Speed_50MHz;  
// GPIO_Init(JDQ_PORT,&JDQ_ENStructure2);    
// 
// 
// GPIO_SetBits(JDQ_PORT,JDQ_PIN11);  
// 
// delay_ms(2000);
// 
// GPIO_SetBits(JDQ_PORT,JDQ_PIN12);   
//}
void LED_Init(void)
{
 GPIO_InitType GPIO_InitStructure;
 GPIO_InitType JDQ_ENStructure1;
 GPIO_InitType JDQ_ENStructure2;
 
 RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE); 
 
 GPIO_InitStructure.Pin = LED_R | LED_G | LED_B | LED_Battery | LED_PIN; 
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
 GPIO_InitPeripheral(LED_PORT, &GPIO_InitStructure);   
 GPIO_ResetBits(LED_PORT, LED_Battery);            
 GPIO_SetBits(LED_PORT, LED_R | LED_G | LED_B);
 GPIO_SetBits(LED_PORT,LED_PIN);
 
 RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE); 
 
 JDQ_ENStructure1.Pin=JDQ_PIN11;  
 JDQ_ENStructure1.GPIO_Mode=GPIO_Mode_Out_PP;  
 JDQ_ENStructure1.GPIO_Speed=GPIO_Speed_50MHz;  
 GPIO_InitPeripheral(JDQ_PORT,&JDQ_ENStructure1);     
 

 JDQ_ENStructure2.Pin=JDQ_PIN12; 
 JDQ_ENStructure2.GPIO_Mode=GPIO_Mode_Out_PP;  
 JDQ_ENStructure2.GPIO_Speed=GPIO_Speed_50MHz;  
 GPIO_InitPeripheral(JDQ_PORT,&JDQ_ENStructure2);    
 
 
 GPIO_SetBits(JDQ_PORT,JDQ_PIN11);  
 
 delay_ms(2000);
 
 GPIO_SetBits(JDQ_PORT,JDQ_PIN12);   
}


/**************************************************************************
函数功能：LED灯闪烁任务、车灯的控制任务
入口参数：无 
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
	while(1)
    {
//      LED=~LED;        //LED状态取反，0是点亮，1是熄灭                                  
//      vTaskDelay(Led_Count); //相对延时函数，500MS改变一次状态
//			Led_LightRGB();
//			Led_LightCar();
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));
			
    }
}  

/**************************************************************************
函数功能：RGB灯点亮显示电量函数
入口参数：无 
返回  值：无
**************************************************************************/
void Led_LightRGB(void)
{
	// 电量灯显示绿灯
	if(g_nVol_get_Flag == 1 && g_fltProprity_Voltage >= 0.5)
	{
		GPIO_SetBits(LED_PORT, LED_R | LED_B);
		GPIO_ResetBits(LED_PORT, LED_G);
	}
	// 电量灯显示黄灯
	else if(g_nVol_get_Flag == 1 && g_fltProprity_Voltage < 0.5 && g_fltProprity_Voltage >= 0.3)
	{
		GPIO_SetBits(LED_PORT, LED_G | LED_R);
		GPIO_ResetBits(LED_PORT, LED_B);
	}
	// 电量灯显示红灯
	else if(g_nVol_get_Flag == 1 && g_fltProprity_Voltage < 0.3 && g_fltProprity_Voltage >= 0.2)
	{
		GPIO_SetBits(LED_PORT, LED_G | LED_B);
		GPIO_ResetBits(LED_PORT, LED_R);
	}
}


/**************************************************************************
函数功能：根据遥控器控制车灯点亮
入口参数：无 
返回  值：无
**************************************************************************/
void Led_LightCar(void)
{
	// 开启车灯标志
	if(g_ucLightOnFlag == 1)
	{
		GPIO_SetBits(LED_PORT, LED_Battery);
		//printf("setbit of GPIOE_Pin_5\r\n");
	}
	else
	{
		GPIO_ResetBits(LED_PORT, LED_Battery);
	}
}

