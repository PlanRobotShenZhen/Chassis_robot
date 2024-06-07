#include "JDQ_EN.h"
#include "delay.h"
#include "rtthread.h"

/*******************************************************************************
* �� �� ��         : JDQ_INIT
* ��������		   : �̵���������ʹ�ܺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
#if(CARMODE != Diff)
void JDQ_INIT()
{
	GPIO_InitType GPIO_ENStructure;//����ṹ�����
	
	RCC_EnableAPB2PeriphClk(JDQ_PORT_RCC, ENABLE);//ʹ�ܶ˿�ʱ��
	
	GPIO_ENStructure.Pin=JDQ1_PIN | JDQ2_PIN ;  //ѡ����Ҫ���õ�IO��
	GPIO_ENStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //�����������ģʽ
	GPIO_ENStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //���ô�������
	GPIO_InitPeripheral(JDQ_PORT,&GPIO_ENStructure); 	   /* ��ʼ��GPIO */

	GPIO_SetBits(JDQ_PORT, JDQ1_PIN);   //��JDQ�˿����ߣ�ʹ��JDQ
	
  rt_thread_delay(5000);   //< 500ms

	GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
}

#endif