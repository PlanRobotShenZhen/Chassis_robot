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
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXDE)==RESET);
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
	NVIC_InitType NVIC_InitStruct;

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
	NVIC_InitType NVIC_InitStruct;

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
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(DMA1_CH4);  // DMA1 ͨ��4, UART1_TX
	DMA_DeInit(DMA1_CH5);  // DMA1 ͨ��5, UART1_RX

	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1, ENABLE);  // ʹ��DMA1��ʱ��

	// ���� DMA1 ͨ��4, UART1_TX
	DMA_InitStruct.PeriphAddr = (USART1_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart1_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(DMA1_CH4, &DMA_InitStruct);


	// ���� DMA1 ͨ��5, UART1_RX
	DMA_InitStruct.PeriphAddr = (USART1_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart1_recv_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
    DMA_InitStruct.BufSize = USART1_RX_MAXBUFF;
    DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
    DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(DMA1_CH5, &DMA_InitStruct);

	USART_EnableDMA(USART1, USART_DMAREQ_TX | USART_DMAREQ_RX, ENABLE);// ʹ��DMA���ڷ��ͺͽ�������

	DMA1_Channel4_nvic_config();

	// ���� DMA1 ͨ��4, UART1_TX ��������ж�
	DMA_ConfigInt(DMA1_CH4, DMA_INT_TXC, ENABLE);

	DMA_EnableChannel(DMA1_CH5, ENABLE);     // ��������
	DMA_EnableChannel(DMA1_CH4, DISABLE);    // ��ֹ����
}


/**
 *  @brief  ����1��ʼ��
 *  @param  ������
 *  @retval ��
 *  @note   UART1_TX -> PA9; UART1_RX -> PA10
 */
void USART1_Init(uint32_t baud)
{
    GPIO_InitType GPIO_InitStruct;
    USART_InitType USART_InitStruct;

	GPIO_DeInit(GPIOA);     // �ָ� GPIOA �ļĴ�����Ĭ��ֵ
	USART_DeInit(USART1);   // �ָ� UART1 �ļĴ�����Ĭ��ֵ

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);     // ʹ�� GPIOA ��ʱ��
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);   // ʹ�� UART1 ��ʱ��

	// ���÷������� PA9 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);

	// ���ý������� PA10 
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);

	// ���ô��ڲ��� �շ�һ�塢8n1��������
    USART_InitStruct.Mode = USART_MODE_RX | USART_MODE_TX;
    USART_InitStruct.BaudRate = baud;
    USART_InitStruct.WordLength = USART_WL_8B;
    USART_InitStruct.StopBits = USART_STPB_1;
    USART_InitStruct.Parity = USART_PE_NO;
    USART_InitStruct.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_Init(USART1, &USART_InitStruct);

	uart1_nvic_config();    // �жϿ���������
	uart1_dma_config();     // DMA����

	USART_ConfigInt(USART1, USART_INT_IDLEF, ENABLE);  // ʹ�ܿ����ж�

	USART_Enable(USART1, ENABLE);  // ʹ�ܴ���1
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
	if (USART_GetIntStatus(USART1, USART_INT_IDLEF) != RESET)
	{
		USART1->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		USART1->DAT; // ��������ж�
		DMA_EnableChannel(DMA1_CH5, DISABLE);//< ��ͣ����
		uart1_recv_flag = 1;                // ���ձ�־��1
		// ͳ���յ������ݵĳ���
		uart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(DMA1_CH5);
	}
} 	

// DMA1 ͨ��4, UART1_TX ��������ж�
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC4, DMA1) != RESET) // DMA1 ͨ��4, UART1_TX �������
	{
		DMA_ClrIntPendingBit(DMA1_INT_TXC4, DMA1);     // ����ж�

		uart1_send_flag = 1;
        DMA_EnableChannel(DMA1_CH4, DISABLE);              // �ر� DMA1 ͨ��4, UART1_TX


		//memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // ��ս��ջ�����
		DMA_EnableChannel(DMA1_CH5, DISABLE);    // DMA1 ͨ��5, UART1_RX
		DMA_SetCurrDataCounter(DMA1_CH5, USART1_RX_MAXBUFF);
        DMA_EnableChannel(DMA1_CH5, ENABLE);      // DMA1 ͨ��5, UART1_RX
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
			DMA_EnableChannel(DMA1_CH4, DISABLE);    // �ر� DMA1 ͨ��4, UART1_TX
			DMA_SetCurrDataCounter(DMA1_CH4, uart1_send_len);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
			DMA_EnableChannel(DMA1_CH4, ENABLE);    // ���� DMA1 ͨ��4, UART1_TX
		}
	}
}

  
#endif	

