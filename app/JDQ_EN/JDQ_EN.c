#include "JDQ_EN.h"
#include "delay.h"
#include "rtthread.h"

/*******************************************************************************
* 函 数 名         : JDQ_INIT
* 函数功能		   : 继电器、车灯使能函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
#if(CARMODE != Diff)
void JDQ_INIT()
{
	GPIO_InitType GPIO_ENStructure;//定义结构体变量
	
	RCC_EnableAPB2PeriphClk(JDQ_PORT_RCC, ENABLE);//使能端口时钟
	
	GPIO_ENStructure.Pin=JDQ1_PIN | JDQ2_PIN ;  //选择你要设置的IO口
	GPIO_ENStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //设置推挽输出模式
	GPIO_ENStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_InitPeripheral(JDQ_PORT,&GPIO_ENStructure); 	   /* 初始化GPIO */

	GPIO_SetBits(JDQ_PORT, JDQ1_PIN);   //将JDQ端口拉高，使能JDQ
	
  rt_thread_delay(5000);   //< 500ms

	GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
}

#endif