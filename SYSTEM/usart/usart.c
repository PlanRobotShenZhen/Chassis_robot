#include "usart.h"	
#include "mb.h"
 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	  
#endif

uint8_t uart1_recv_data[USART1_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t uart1_send_data[USART1_TX_MAXBUFF] = { 0 };  // �������ݻ�����
uint32_t uart1_recv_flag = 0;        // ������ɱ�־λ
uint32_t uart1_recv_len = 0;         // ���յ����ݳ���
uint32_t uart1_send_len = 0;         // ���͵����ݳ���
uint32_t uart1_send_flag = 0;        // ������ɱ�־λ

/*****�������´���,֧��printf����,������Ҫѡ��use MicroLIB****/	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch,FILE *p)  //����Ĭ�ϵģ���ʹ��printf����ʱ�Զ�����
{
	USART_SendData(USART1,(u8)ch);	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

#endif 
/************************************************************/

/**
 *  @brief  MA1 ͨ��4, UART1_TX �жϿ���������
 *  @param  ��
 *  @retval ��
 *  @note   �ж����ȼ�����ȫ����ֻ����һ�Σ��� main �����ʼ��������
 *  @note   �жϴ������� CMSIS/stm32f10x_it.c �н��д���
 */
void DMA1_Channel4_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	// ���ô���1���жϿ�����
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // �� stm32f10x.h ���� IRQn_Type ö��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  ����1�жϿ���������
 *  @param  ��
 *  @retval ��
 *  @note   �ж����ȼ�����ȫ����ֻ����һ�Σ��� main �����ʼ��������
 *  @note   �жϴ������� CMSIS/stm32f10x_it.c �н��д���
 */
void uart1_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	// ���ô���1���жϿ�����
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;          // �� stm32f10x.h ���� IRQn_Type ö��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  ����1 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note    UART1_TX -> DMA1 Channel4; UART1_RX -> DMA1 Channel5
 */
void uart1_dma_config(void)
{
	DMA_InitTypeDef DMA_InitStruct;

	DMA_DeInit(DMA1_Channel4);  // DMA1 ͨ��4, UART1_TX
	DMA_DeInit(DMA1_Channel5);  // DMA1 ͨ��5, UART1_RX

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // ʹ��DMA1��ʱ��

	// ���� DMA1 ͨ��4, UART1_TX
	DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_send_data;  // �ڴ��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStruct);

	// ���� DMA1 ͨ��5, UART1_RX
	DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_recv_data;  // �ڴ��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                 // ���赽�ڴ�
	DMA_InitStruct.DMA_BufferSize = USART1_RX_MAXBUFF;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStruct);

	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);// ʹ��DMA���ڷ��ͺͽ�������

	DMA1_Channel4_nvic_config();

	// ���� DMA1 ͨ��4, UART1_TX ��������ж�
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Channel5, ENABLE);     // ��������
	DMA_Cmd(DMA1_Channel4, DISABLE);    // ��ֹ����
}


/**
 *  @brief  ����1��ʼ��
 *  @param  ������
 *  @retval ��
 *  @note   UART1_TX -> PA9; UART1_RX -> PA10
 */
void USART1_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	GPIO_DeInit(GPIOA);     // �ָ� GPIOA �ļĴ�����Ĭ��ֵ
	USART_DeInit(USART1);   // �ָ� UART1 �ļĴ�����Ĭ��ֵ

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // ʹ�� GPIOA ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   // ʹ�� UART1 ��ʱ��

	// ���÷������� PA9 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// ���ý������� PA10 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// ���ô��ڲ��� �շ�һ�塢8n1��������
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_BaudRate = baud;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStruct);

	uart1_nvic_config();    // �жϿ���������
	uart1_dma_config();     // DMA����

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // ʹ�ܿ����ж�

	USART_Cmd(USART1, ENABLE);  // ʹ�ܴ���1
}


#if EN_USART1_RX   //���ʹ���˽���
/*******************************************************************************
* �� �� ��         : USART1_IRQHandler
* ��������		   : USART1�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/ 
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART1->SR; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		USART1->DR; // ��������ж�
		DMA_Cmd(DMA1_Channel5, DISABLE);//< ��ͣ����
		uart1_recv_flag = 1;                // ���ձ�־��1
		// ͳ���յ������ݵĳ���
		uart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(DMA1_Channel5);
	}
} 	

// DMA1 ͨ��4, UART1_TX ��������ж�
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC4) != RESET) // DMA1 ͨ��4, UART1_TX �������
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);     // ����ж�

		uart1_send_flag = 1;
		DMA_Cmd(DMA1_Channel4, DISABLE);        // �ر� DMA1 ͨ��4, UART1_TX


		//memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // ��ս��ջ�����
		DMA_Cmd(DMA1_Channel5, DISABLE);    // DMA1 ͨ��5, UART1_RX
		DMA_SetCurrDataCounter(DMA1_Channel5, USART1_RX_MAXBUFF);
		DMA_Cmd(DMA1_Channel5, ENABLE);     // DMA1 ͨ��5, UART1_RX
	}
}
void modbus_task_init(void)
{
	UCHAR mySlaveAddress = 0x01;//< �ӻ���ַ
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
}
void modbus_task(void* pvParameters)
{
	TickType_t lastWakeTime = (TickType_t)getSysTickCnt();
	/* Enable the Modbus Protocol Stack. */
	eMBEnable();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, 1);//��������1000Hz��Ƶ������
		if (uart1_recv_flag == 1)
		{
			uart1_recv_flag = 0;
			pxMBFrameCBByteReceived();
		}
		eMBPoll();
		if (uart1_send_flag == 2)
		{//< �ظ�֡
			uart1_send_flag = 0;
			DMA_Cmd(DMA1_Channel4, DISABLE);    // �ر� DMA1 ͨ��4, UART1_TX
			DMA_SetCurrDataCounter(DMA1_Channel4, uart1_send_len);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
			DMA_Cmd(DMA1_Channel4, ENABLE);    // ���� DMA1 ͨ��4, UART1_TX
		}
	}
}

  
#endif	

