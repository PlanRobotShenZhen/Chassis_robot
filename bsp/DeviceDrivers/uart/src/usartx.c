#include "usartx.h"
#include "user_can.h"
#include "485_address.h"
#include "mb.h"
#include "robot_select_init.h"
#include "balance.h"
#include "bsp.h"
#include "led.h"
#include "motor_data.h"
#include "FLASH_WR.h"
#include "rc_car.h"
#include "remote.h"
#include "motor.h"
void Modbus_Respond(MBModify* modify);
/*--------����λ��ͨ�Ÿ�ʽ����usart3-----------*/
SEND_DATA Send_Data;//�������ݵĽṹ��
unsigned char Receive_Data[RECEIVE_DATA_SIZE] = {0};//�������ݵĽṹ��
unsigned char Receive_Null[RECEIVE_DATA_SIZE] = {0};
/*----------------------------usart1��3��5----------------------------*/
uint8_t usart1_send_data[USART1_TX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t usart1_send_len = 0;         // ���͵����ݳ���
uint8_t usart1_send_flag = 0;        // ������ɱ�־λ

uint8_t usart1_recv_data[USART1_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t usart1_recv_flag = 0;        // ������ɱ�־λ
uint8_t usart1_recv_len = 0;         // ���յ����ݳ���

uint8_t usart3_send_data[USART3_TX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t usart3_send_len = 0;         // ���͵����ݳ���
uint8_t usart3_send_flag = 1;        // ������ɱ�־λ

uint8_t usart3_recv_data[USART3_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t usart3_recv_flag = 0;        // ������ɱ�־λ
uint8_t usart3_recv_len = 0;         // ���յ����ݳ���

uint8_t uart4_send_data[UART4_TX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t uart4_send_len = 0;         // ���͵����ݳ���
uint8_t uart4_send_flag = 0;        // ������ɱ�־λ

uint8_t uart4_recv_data[UART4_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t uart4_recv_flag = 0;        // ������ɱ�־λ
uint8_t uart4_recv_len = 0;         // ���յ����ݳ���

unsigned char Uart5_Buffer[Max_BUFF_Len] = { 0 }; 

bool ROS_RecvFlag = false;
bool Motor_Enable_Flag = false;
int ROS_Count = 0;
uint8_t CheckSum_sdo[1][8] = { 0 }; 

/*******************************************************************************USART1*******************************************************************************/
/*******************************************************************************
* �� �� ��: Usart1_Init
* ��������: USART1��ʼ������
* ��    ��: bound:������
* ��    ��: ��
*******************************************************************************/ 
void Usart1_Init(uint32_t baud)
{
	GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStruct;
	RCC_EnableAPB2PeriphClk(USARTOne_GPIO_CLK | USARTOne_CLK, ENABLE);

	GPIO_InitStructure.Pin = USARTOne_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitPeripheral(USARTOne_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = USARTOne_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitPeripheral(USARTOne_GPIO, &GPIO_InitStructure);

	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(USARTOne, &USART_InitStructure);

	NVIC_InitStruct.NVIC_IRQChannel = USARTOne_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_ConfigInt(USARTOne, USART_INT_IDLEF, ENABLE);  // ʹ�ܿ����ж�
	Usart1_Dma_Config();     // DMA����
	USART_Enable(USARTOne, ENABLE);
}

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
/*******************************************************************************
 *  @brief  ����1 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note    UART1_TX -> DMA1 Channel4; UART1_RX -> DMA1 Channel5
*******************************************************************************/
void Usart1_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(USARTOne_Tx_DMA_Channel);  // DMA1 ͨ��4, UART1_TX
	DMA_DeInit(USARTOne_Rx_DMA_Channel);  // DMA1 ͨ��5, UART1_RX

	RCC_EnableAHBPeriphClk(USARTOne_DMAx_CLK, ENABLE);

	// ���� DMA1 ͨ��4, UART1_TX
	DMA_InitStruct.PeriphAddr = USARTOne_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart1_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTOne_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART1_TX, DMA1, USARTOne_Tx_DMA_Channel, ENABLE);

	// ���� DMA1 ͨ��5, UART1_RX
	DMA_InitStruct.PeriphAddr = USARTOne_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart1_recv_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
	DMA_InitStruct.BufSize = USART1_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTOne_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART1_RX, DMA1, USARTOne_Rx_DMA_Channel, ENABLE);

	// ���ô���1���жϿ�����
	DMA1_Channel4_nvic_config();

	DMA_ConfigInt(USARTOne_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);// ���� DMA1 ͨ��4, UART1_TX ��������ж�
	USART_EnableDMA(USARTOne, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);// ʹ��DMA���ڷ��ͺͽ�������

	DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);     // ��������
	DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);    // ��ֹ����
}

/*******************************************************************************
* �� �� ��         : USART1_IRQHandler
* ��������		   : USART1�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void USARTOne_IRQHandler(void)                	//����1�жϷ������
{
	if (USART_GetIntStatus(USARTOne, USART_INT_IDLEF) != RESET){
		USARTOne->STS;
		USARTOne->DAT;
		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);     // ��ͣ����
		usart1_recv_flag = 1;                // ���ձ�־��1
		usart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTOne_Rx_DMA_Channel);// ͳ���յ������ݵĳ���
	}
}

/*******************************************************************************
* ��������		   	 : USART1_TX ��������ж�
* ��    ��         : ��
* ˵    ��         : DMA1 ͨ��4, UART1_TX ��������ж�
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC4, DMA1)){ // DMA1 ͨ��4, UART1_TX �������
		DMA_ClrIntPendingBit(DMA1_INT_TXC4, DMA1);      // ����ж�
		DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);    // �ر� DMA1 ͨ��4, UART1_TX

//		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);    // DMA1 ͨ��5, UART1_RX
//		DMA_SetCurrDataCounter(USARTOne_Rx_DMA_Channel, USART1_RX_MAXBUFF);
//		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);      // DMA1 ͨ��5, UART1_RX
	}
}

/*******************************************************************************
* ��������		     : modbus��ʼ������
* ��    ��         : ��
* ˵    ��         :��
*******************************************************************************/ 

void modbus_task_init(void)
{
	UCHAR mySlaveAddress = 0x01;//< �ӻ���ַ
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);//�����ֱ�Ϊmodbus�Ĺ���ģʽ���ӻ���ַ���˿ںš������ʡ���żУ�����á�
	MyFLASH_ReadByte(FINAL_PAGE_ADDRESS,pdu , MB_RTU_DATA_MAX_SIZE);
	
	if (pdu[car_type] == 0xFFFF)
		{//< оƬ�״γ�ʼ��
		eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
		Pdu_Init();
		MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		pdu[para_save] = 10;
	}
	else 
		pdu[para_save] = 0;
}

/*******************************************************************************
* ��������: USART1��������
* ��    ��: ��
* ˵    ��: UART1_TXͨ��DMA1ͨ��4��usart1_send_data�����ݴ���������ΪmodbusЭ���ʽ��֡��
*******************************************************************************/
void Modbus_task(void* pvParameters)
{
	MBModify modify;
	/* Enable the Modbus Protocol Stack. */
	eMBEnable();
	while (1){
		rt_thread_delay(10);   //< 1ms
		if (usart1_recv_flag){
			LedBlink(LED17_GPIO, LED17_PIN);
			usart1_recv_flag = 0;
			pxMBFrameCBByteReceived();
		}
		eMBPoll(&modify);
		if (usart1_send_flag){//< �ظ�֡
			DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);   // �ر� DMA1 ͨ��4, UART1_TX
			DMA_SetCurrDataCounter(USARTOne_Tx_DMA_Channel, usart1_send_len);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
			DMA_EnableChannel(USARTOne_Tx_DMA_Channel, ENABLE);    // ���� DMA1 ͨ��4, UART1_TX
			usart1_send_flag = 0;
			// ������ɻָ�����
			DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);    // DMA1 ͨ��5, UART1_RX
			DMA_SetCurrDataCounter(USARTOne_Rx_DMA_Channel, USART1_RX_MAXBUFF);
			DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);     // DMA1 ͨ��5, UART1_RX
			Modbus_Respond(&modify);
		}
		if (pdu[para_save] == 1){//< ���浱ǰ����
			pdu[para_save] = 0;
			MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		}else if (pdu[para_save] == 2){//< �ָ���������
			pdu[para_save] = 0;
			eMBInit(MB_RTU, 1, 3, 115200, MB_PAR_NONE);
			Pdu_Init();
			MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		}
		if (pdu[software_reset] == 0xA5){
			pdu[software_reset] = 0;
			pdu[car_error_messages] = 0;
			Soft_Reset();
		}else if (pdu[software_reset] == 0x5A){
			pdu[software_reset] = 0;
			Jump_To_BOOT();
		}
		if (pdu[error_clearance] == 1){
			pdu[error_clearance] = 0;
			pdu[car_error_messages] = 0;
		}
	}
}
/*******************************************************************************USART2*******************************************************************************/
/**************************************************************************
�������ܣ�Բ�ε��̴���2��ʼ�������洮��4��Ϊ��ȡ�����Ϣģ��
��ڲ�����baudΪͨ�Ų�����
�� �� ֵ����
**************************************************************************/
void Usart2_Init(uint32_t baud)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	/* System Clocks Configuration */
	RCC_EnableAPB1PeriphClk(USARTTwo_CLK, ENABLE);	//ʹ��USART2ʱ��
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | USARTTwo_GPIO_CLK, ENABLE);//ʹ��GPIOʱ��
	GPIO_ConfigPinRemap(GPIO_RMP2_USART2, ENABLE);

	//USART2_TX  
	GPIO_InitStructure.Pin = USARTTwo_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	//USART2_RX	  
	GPIO_InitStructure.Pin = USARTTwo_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	//485 enable	  
	GPIO_InitStructure.Pin = USARTTwo_CTS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�����������
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	/* USARTTwo configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(USARTTwo, &USART_InitStructure);

	//UartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USARTTwo_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ����ж����ȼ����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	USART2_DMA_Config();     // DMA����
	USART_ConfigInt(USARTTwo, USART_INT_IDLEF, ENABLE);  // ʹ�ܿ����ж�

	USART_Enable(USARTTwo, ENABLE);
}

/*******************************************************************************
 *  @brief  ����2 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note     USART2_TX-> DMA1 Channe07; USART2_RX-> DMA1 Channe06
*******************************************************************************/
void USART2_DMA_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(USARTTwo_Tx_DMA_Channel);  // DMA1 ͨ��7, USART2_TX
	DMA_DeInit(USARTTwo_Rx_DMA_Channel);  // DMA1 ͨ��6, USART2_RX
	DMA_StructInit(&DMA_InitStruct);
	RCC_EnableAHBPeriphClk(USARTTwo_DMAx_CLK, ENABLE);

	DMA_InitStruct.PeriphAddr = USARTTwo_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;

	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTTwo_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART2_TX, DMA1, USARTTwo_Tx_DMA_Channel, ENABLE);

	// ���� DMA1 ͨ��6, USART2_RX
	DMA_InitStruct.PeriphAddr = USARTTwo_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_recv_data;  // �ڴ��ַ,��UART4��ͳһ����
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
	DMA_InitStruct.BufSize = UART4_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTTwo_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART2_RX, DMA1, USARTTwo_Rx_DMA_Channel, ENABLE);

	// ���ô���2��DMA�����ж�
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; 	// ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);

	// ���� DMA2 ͨ��5, UART4_TX ��������ж�
	DMA_ConfigInt(USARTTwo_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(USARTTwo, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);     // ��ֹ����
	DMA_EnableChannel(USARTTwo_Tx_DMA_Channel, DISABLE);     // ��ֹ����
}

/*******************************************************************************
* ��������: USART2_TX ��������ж�
* ��    ��: ��
* ˵    ��: DMA1 ͨ��7, USART2_TX ��������ж�
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC7, DMA1) != RESET) { // DMA1 ͨ��7, USART2_TX �������
		DMA_ClrIntPendingBit(DMA1_INT_TXC7, DMA1);        // ����ж�
		uart4_send_flag = 1;
		GPIO_ResetBits(USARTTwo_GPIO, USARTTwo_CTS);
		DMA_EnableChannel(USARTTwo_Tx_DMA_Channel, DISABLE);    // �ر� DMA2 ͨ��5, UART4_TX
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);    // DMA2 ͨ��3, UART4_RX
		DMA_SetCurrDataCounter(USARTTwo_Rx_DMA_Channel, UART4_RX_MAXBUFF);
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, ENABLE);     // DMA2 ͨ��3, UART4_RX
	}
}

/**************************************************************************
* �������ܣ�����2�����ж�,���յ�ط��͹���������
* ��ڲ�������
* �� �� ֵ����
**************************************************************************/
void USARTTwo_IRQHandler(void)
{
	if (USART_GetIntStatus(USARTTwo, USART_INT_IDLEF) != RESET) {
		USARTTwo->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		USARTTwo->DAT; // ��������ж�
		//Ϊ������������С��һ�£��˴��ô���4�����ı���
		uart4_recv_flag = 1;                 // ���ձ�־��1
		uart4_recv_len = UART4_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTTwo_Rx_DMA_Channel);// ͳ���յ������ݵĳ���
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);    // DMA1 ͨ��6, USART2_RX 
	}
}

/*******************************************************************************USART3*******************************************************************************/
/**************************************************************************
�������ܣ�����3��ʼ������Ϊ����λ��ͨ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Usart3_Init(u32 baud)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	RCC_EnableAPB2PeriphClk(USARTThree_GPIO_CLK, ENABLE);	//ʹ��GPIOʱ��
	RCC_EnableAPB1PeriphClk(USARTThree_CLK, ENABLE);	//ʹ��USART3ʱ��
#if (CARMODE == Diff)
	{
		//Բ�ε������Ÿ���
		RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
		GPIO_ConfigPinRemap(GPIO_PART_RMP_USART3, ENABLE);
	}
#endif
	//USART3_TX  
	GPIO_InitStructure.Pin = USARTThree_TxPin; //PB10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitPeripheral(USARTThree_GPIO, &GPIO_InitStructure);

	//USART3_RX	  
	GPIO_InitStructure.Pin = USARTThree_RxPin;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_InitPeripheral(USARTThree_GPIO, &GPIO_InitStructure);

	/* USART3 configuration*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(USARTThree, &USART_InitStructure);

	//UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USARTThree_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ����ж����ȼ����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	USART_ConfigInt(USARTThree, USART_INT_IDLEF, ENABLE);  // ʹ�ܿ����ж�
	Usart3_dma_config();     // DMA����	

	USART_Enable(USARTThree, ENABLE);
}

/*******************************************************************************
 *  @brief  ����3 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note    USART3_TX -> DMA1 Channel2; USART3_RX -> DMA1 Channel3
*******************************************************************************/
void Usart3_dma_config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	RCC_EnableAHBPeriphClk(USARTThree_DMAx_CLK, ENABLE);

	// ���� DMA1 ͨ��2, USART3_TX
	DMA_InitStruct.PeriphAddr = USARTThree_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart3_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTThree_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART3_TX, DMA1, USARTThree_Tx_DMA_Channel, ENABLE);

	// ���� DMA1 ͨ��3, USART3_RX
	DMA_InitStruct.PeriphAddr = USARTThree_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart3_recv_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
	DMA_InitStruct.BufSize = USART3_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTThree_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART3_RX, DMA1, USARTThree_Rx_DMA_Channel, ENABLE);

	// ���ô���3���жϿ�����
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;   // �� stm32f10x.h ���� IRQn_Type ö��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);

	// ���� DMA1 ͨ��4, USART3_TX ��������ж�
	DMA_ConfigInt(USARTThree_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(USARTThree, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(USARTThree_Rx_DMA_Channel, ENABLE);     // ��������
	DMA_EnableChannel(USARTThree_Tx_DMA_Channel, DISABLE);    // ��ֹ����
}

/**************************************************************************
* ��������		   	 : USART3_RX ��������ж�
* ��    ��         : ��
* ˵    ��         : DMA1 ͨ��3, USART3_RX ��������ж�
**************************************************************************/
void USARTThree_IRQHandler(void)
{	
	if (USART_GetIntStatus(USARTThree, USART_INT_IDLEF)){
		USART3->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		USART3->DAT; // ��������ж�
		DMA_EnableChannel(USARTThree_Rx_DMA_Channel, DISABLE);				//��ͣ����
		usart3_recv_len = USART3_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTThree_Rx_DMA_Channel);
		usart3_recv_flag = 1;
	}
}

/*******************************************************************************
* ��������: USART3_TX ��������ж�
* ��    ��: ��
* ˵    ��: DMA1 ͨ��2, UART3_TX ��������ж�
*******************************************************************************/ 
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC2, DMA1)){ //�ȴ�DMA�������, UART3_TX ��������ж�Ϊ1
		DMA_ClrIntPendingBit(DMA1_INT_TXC2, DMA1);     // ����ж�
		DMA_EnableChannel(USARTThree_Tx_DMA_Channel, DISABLE);        // �ر� DMA1 ͨ��2, UART3_TX
		usart3_send_flag = 1;
	}
}

/**************************************************************************
�������ܣ�������λ�����͹���������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Usart3_Recv(void)
{
	if(ROS_Count < 10000){//��ֹ�������
		ROS_Count++;
	}
	if((pdu[control_mode] != control_mode_remote)){						
		if(usart3_recv_flag){	
			usart3_recv_flag = 0;
			rt_memcpy(Receive_Data, usart3_recv_data, RECEIVE_DATA_SIZE);	
			u8 temp11 = Check_Sum(RECEIVE_DATA_SIZE - 2,0);
			if ((usart3_recv_data[0] == FRAME_HEADER) &&
				(usart3_recv_data[RECEIVE_DATA_SIZE - 1] == FRAME_TAIL) &&
				(Receive_Data[RECEIVE_DATA_SIZE -2] == Check_Sum(RECEIVE_DATA_SIZE - 2, 0))){	//��֤���ݰ��ĳ��ȡ�֡ͷ֡β
				pdu[control_mode] = control_mode_ros;	
				ROS_Count = 0;
				ROS_RecvFlag = true;
				robot_control.ctrl = Receive_Data[1];
			}else{		
				CheckSum_sdo[0][0] = Check_Sum(RECEIVE_DATA_SIZE - 2, 0);
				Add_Sdo_Linked_List(CHECKSUM_ID, CheckSum_sdo, sizeof(CheckSum_sdo)/sizeof(CheckSum_sdo[0]));//ͨ��CAN��ʾ��ȡ��Sum
				if(ROS_Count > 300)	pdu[control_mode] = control_mode_unknown;//1500msδ���յ���ȷros��Ϣ�����������ģʽ
			}
			DMA_SetCurrDataCounter(USARTThree_Rx_DMA_Channel, USART3_RX_MAXBUFF);//���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
			DMA_EnableChannel(USARTThree_Rx_DMA_Channel, ENABLE);     		// DMA1 ͨ��2, UART3_RX
		}else if(ROS_Count > 300){	//1500msδ���յ�ros��Ϣ�����������ģʽ
			pdu[control_mode] = control_mode_unknown;
		}
	}
}

/**************************************************************************
�������ܣ�����3(ROS)��������
��ڲ�������
����  ֵ����
**************************************************************************/
void Usart3_Send(void)
{
	if (usart3_send_flag){
		usart3_send_flag = 0;	
		
		Data_Assignment(); 
		
		rt_memcpy(usart3_send_data, Send_Data.buffer, USART3_TX_MAXBUFF);
		DMA_SetCurrDataCounter(USARTThree_Tx_DMA_Channel, USART3_TX_MAXBUFF); 
		DMA_EnableChannel(USARTThree_Tx_DMA_Channel, ENABLE);    // ����DMA1 ͨ��2��USART3_TX	
	}
}

/**************************************************************************
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void Data_Assignment(void)
{
	Send_Data.d.Frame_Header = FRAME_HEADER;			//1���ֽ� = FRAME_HEADER; //֡ͷ(�̶�ֵ)      		
	Send_Data.d.Motor_Enable_Flag = Motor_Enable_Flag;
	
	Send_Data.d.X_speed = pdu[linear_speed_feedback];	  
	Send_Data.d.Z_speed = pdu[yaw_speed_feedback];    
	
	Send_Data.d.Power_Quantity = pdu[BatteryQuantity]; 
	Send_Data.d.Power_Voltage = pdu[BatteryVoltage]; 
	Send_Data.d.Power_Current = pdu[BatteryCurrent];    
	Send_Data.d.Power_Temperature = pdu[BatteryTemperature];    
	Send_Data.d.M1_current = 0;    
	Send_Data.d.M2_current = 0;      		
	Send_Data.d.Odometry1 = pdu[Odom1ForRobot];
	Send_Data.d.Odometry2 = pdu[Odom2ForRobot];       	//2���ֽ�	
	Send_Data.d.Check_SUM = Check_Sum(USART3_TX_MAXBUFF-2, 1);;
	Send_Data.d.Frame_Tail = FRAME_TAIL; //֡β���̶�ֵ��
}

/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����23λΪУ��λ�����������1-22�����������һ������Ϊ���ͻ��ǽ���У��λ
�� �� ֵ������λ
**************************************************************************/
unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode) 
{
	unsigned char check_sum = 0, k;
	// ���ݻ�������ѡ��
	unsigned char* dataBuffer = (Mode == 1) ? Send_Data.buffer : Receive_Data;
	for (k = 0; k < Count_Number; k++) {check_sum = check_sum ^ dataBuffer[k];}
	return check_sum;
}

/**************************************************************************
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short������
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
short ConvertBytesToShort(u8 highByte,u8 lowByte)
{
    return (((short)highByte << 8) + (short)lowByte);    
}

/*******************************************************************************UART4*******************************************************************************/
/**************************************************************************
�������ܣ�����4��ʼ������Ϊ��ȡ�����Ϣģ��
��ڲ�����baudΪͨ�Ų�����
�� �� ֵ����
**************************************************************************/
#if(CARMODE != Diff)
void Uart4_Init(uint32_t baud)         
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	/* System Clocks Configuration */
	RCC_EnableAPB1PeriphClk(UARTFour_CLK, ENABLE);	//ʹ��UART4ʱ��
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | UARTFour_GPIO_CLK, ENABLE);//ʹ��GPIOʱ��
	
	/* Configure the GPIO ports */
	GPIO_ConfigPinRemap(AFIO_RMP_CFG_SW_JTAG_CFG_DISABLE | GPIO_RMP_SW_JTAG_DISABLE, ENABLE);

	//UART4_TX  
	GPIO_InitStructure.Pin = UARTFour_TxPin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitPeripheral(UARTFour_GPIO, &GPIO_InitStructure);

	//UART4_RX	  
	GPIO_InitStructure.Pin = UARTFour_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitPeripheral(UARTFour_GPIO, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_RMP2_UART4, ENABLE);

	//485 enable	  
	GPIO_InitStructure.Pin = UARTFour_485enPin; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�����������
	GPIO_InitPeripheral(UARTFour_485en_GPIO, &GPIO_InitStructure);

	/* UARTFour configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(UARTFour, &USART_InitStructure);

	//UartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ����ж����ȼ����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	Uart4_Dma_Config();     // DMA����
	USART_ConfigInt(UARTFour, USART_INT_IDLEF, ENABLE);  // ʹ�ܿ����ж�

	USART_Enable(UARTFour, ENABLE);
}

/*******************************************************************************
 *  @brief  ����4 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note    UART4_TX -> DMA2_CH5; UART3_RX -> DMA2_CH3
*******************************************************************************/
void Uart4_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(UARTFour_Tx_DMA_Channel);  // DMA2 ͨ��5, UART4_TX
	DMA_DeInit(UARTFour_Rx_DMA_Channel);  // DMA2 ͨ��3, UART4_RX
	DMA_StructInit(&DMA_InitStruct);
	RCC_EnableAHBPeriphClk(UARTFour_DMAx_CLK, ENABLE);

	// ���� DMA2 ͨ��5, UART4_TX
	DMA_InitStruct.PeriphAddr = UARTFour_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
 
	DMA_Init(UARTFour_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA2_REMAP_UART4_TX, DMA2, UARTFour_Tx_DMA_Channel, ENABLE);

	// ���� DMA2 ͨ��3, UART4_RX
	DMA_InitStruct.PeriphAddr = UARTFour_DR_Base;   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_recv_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
	DMA_InitStruct.BufSize = UART4_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(UARTFour_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA2_REMAP_UART4_RX, DMA2, UARTFour_Rx_DMA_Channel, ENABLE);

	// ���ô���4��DMA�����ж�
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Channel5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; 	// ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);

	// ���� DMA2 ͨ��5, UART4_TX ��������ж�
	DMA_ConfigInt(UARTFour_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(UARTFour, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);     // ��ֹ����
	DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);     // ��ֹ����
}

/*******************************************************************************
* ��������: UART4_TX ��������ж�
* ��    ��: ��
* ˵    ��: DMA1 ͨ��2, UART4_TX ��������ж�
*******************************************************************************/ 
void DMA2_Channel5_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA2_INT_TXC5, DMA2) != RESET){ // DMA2 ͨ��5, UART4_TX �������
		DMA_ClrIntPendingBit(DMA2_INT_TXC5, DMA2);     // ����ж�
		uart4_send_flag = 1;
		GPIO_ResetBits(UARTFour_485en_GPIO, UARTFour_485enPin);
		DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);    // �ر� DMA2 ͨ��5, UART4_TX
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);    // DMA2 ͨ��3, UART4_RX
		DMA_SetCurrDataCounter(UARTFour_Rx_DMA_Channel, UART4_RX_MAXBUFF);
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, ENABLE);     // DMA2 ͨ��3, UART4_RX
	}
}

/**************************************************************************
* �������ܣ�����4�����ж�,���յ�ط��͹���������
* ��ڲ�������
* �� �� ֵ����
**************************************************************************/
void UARTFour_IRQHandler(void)
{
	if (USART_GetIntStatus(UARTFour, USART_INT_IDLEF) != RESET){
		UARTFour->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		UARTFour->DAT; // ��������ж�
		uart4_recv_flag = 1;                 // ���ձ�־��1
		uart4_recv_len = UART4_RX_MAXBUFF - DMA_GetCurrDataCounter(UARTFour_Rx_DMA_Channel);// ͳ���յ������ݵĳ���
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);    // DMA2 ͨ��3, UART4_RX
	}
}
#endif
/**************************************************************************
* �������ܣ�JTAGģʽ����,��������JTAG��ģʽ
* ��ڲ�����mode: jtag,swdģʽ����;00, ȫʹ��;01, ʹ��SWD;10, ȫ�ر�;
* ˵   �� ��JTAG,SWD��û��NJTRSTѡ��û��д��  
**************************************************************************/	  
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2PCLKEN |= 1 << 0;     //开�?辅助时钟	   
	AFIO->RMP_CFG &= 0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->RMP_CFG |= temp;       //设置jtag模式
}

/*******************************************************************************UART5*******************************************************************************/
/**************************************************
* �������ܣ�	����5��ʼ����������Ϊ��ģ�źŽ���
* ��    ����  unBound����ͨ�Ų����ʣ���Ϊ100k
* �� �� ֵ��  ��
* 100k�����ʣ�8λ����λ(stm32-ѡ��9λ)��2λֹͣλ��żУ�飨EVEN)���޿�����25���ֽڡ�
**************************************************/
void Uart5_Init(unsigned int unBound)
{
	//GPIO�˿�����
  	GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStructure;	

	RCC_EnableAPB1PeriphClk(UARTFive_CLK, ENABLE);	//ʹ��UART5��GPIOAʱ��
  	RCC_EnableAPB2PeriphClk(UARTFive_GPIO_CLK, ENABLE);

  	//UART5_RX	  GPIOB.14��ʼ��
  	GPIO_InitStructure.Pin = UARTFive_RxPin;//PB14
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  	GPIO_InitPeripheral(UARTFive_GPIO, &GPIO_InitStructure);//��ʼ��GPIOB.14  
#if (CARMODE != Diff)
	{
		GPIO_ConfigPinRemap(GPIO_RMP1_UART5, ENABLE);
	}
#endif

   	//UART ��ʼ������
	USART_InitStructure.BaudRate = unBound;//���ڲ�����
	USART_InitStructure.WordLength = USART_WL_9B;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.StopBits = USART_STPB_2;//2��ֹͣλ
	USART_InitStructure.Parity = USART_PE_NO;//żУ��λ
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;//��Ӳ������������
	USART_InitStructure.Mode = USART_MODE_RX;	//��ģʽ
  	USART_Init(UARTFive, &USART_InitStructure); //��ʼ������5

  	//UART5 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UARTFive_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���  
	Uart5_Dma_Config();     // DMA����
	USART_ConfigInt(UARTFive, USART_INT_IDLEF, ENABLE);//�������ڿ����ж�
//	//test
//	USART_ConfigInt(UARTFive, USART_INT_OREF, ENABLE);//������������ж�
//	USART_ConfigInt(UARTFive, USART_INT_ERRF, ENABLE);//�������ڴ����ж�
//	USART_ConfigInt(UARTFive, USART_INT_NEF, ENABLE);//�������ڴ����ж�
//	USART_ConfigInt(UARTFive, USART_INT_FEF, ENABLE);//�������ڴ����ж�

	USART_Enable(UARTFive, ENABLE);                    //ʹ�ܴ���5 
}

/**************************************************
* �������ܣ�	����5��ʼ����������Ϊ��ģ�źŽ���
* ��    ����  unBound����ͨ�Ų����ʣ���Ϊ100k
* �� �� ֵ��  ��
**************************************************/
void Uart5_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(UARTFive_Rx_DMA_Channel);  // DMA1 ͨ��8, UART5_RX
	DMA_StructInit(&DMA_InitStruct);
	// ���� DMA1 ͨ��8, UART5_RX
	DMA_InitStruct.PeriphAddr = UARTFive_DR_Base;   	// ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)Uart5_Buffer;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;    // ���赽�ڴ�
	DMA_InitStruct.BufSize = Max_BUFF_Len;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(UARTFive_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_UART5_RX, DMA1, UARTFive_Rx_DMA_Channel, ENABLE);

	USART_EnableDMA(UARTFive, USART_DMAREQ_RX, ENABLE);

	DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // ��������
}

/**************************************************
* �������ܣ�	�źŽ����жϴ�����
* ��ڲ�����  ��
* �� �� ֵ��  ��
**************************************************/
void UARTFive_IRQHandler()
{
	if (USART_GetIntStatus(UARTFive, USART_INT_IDLEF)) //�����жϲ��� 
	{
		UARTFive->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		UARTFive->DAT; // ��������ж�
		Sbus_Data_Parsing_Flag = 1;                // ���ձ�־��1
		pdu[control_mode] = control_mode_remote;                // ��ģ����
		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, DISABLE);    // DMA1 ͨ��3, UART3_RX
		DMA_SetCurrDataCounter(UARTFive_Rx_DMA_Channel, UART5_RX_MAXBUFF);
		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // DMA1 ͨ��3, UART3_RX
	}
//	if (USART_GetIntStatus(UARTFive, USART_INT_ERRF) != RESET)
//	{// USART_FLAG_ERRF
//		USART_ClrFlag(UARTFive, USART_INT_ERRF);
//		USART_ReceiveData(UARTFive);
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_OREF) != RESET)
//	{// USART_FLAG_ORE
//		USART_ClrFlag(UARTFive, USART_INT_OREF);
//		USART_ReceiveData(UARTFive);
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_IDLEF) != RESET) //�жϲ��� 
//	{
//		UARTFive->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
//		UARTFive->DAT; // ��������ж�
//		Sbus_Data_Parsing_Flag = 1;                // ���ձ�־��1
//		pdu[control_mode] = control_mode_remote;   // ��ģ����
//		// ͳ���յ������ݵĳ���
//		//ucRcvCount = Max_BUFF_Len - DMA_GetCurrDataCounter(UARTFive_Rx_DMA_Channel);
//		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, DISABLE);    // DMA1 ͨ��3, UART3_RX
//		DMA_SetCurrDataCounter(UARTFive_Rx_DMA_Channel, Max_BUFF_Len);
//		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // DMA1 ͨ��3, UART3_RX
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_NEF) != RESET)
//	{
//		UARTFive->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
//		UARTFive->DAT; // ��������ж�
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_FEF) != RESET)
//	{
//		UARTFive->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
//		UARTFive->DAT; // ��������ж�
//	}
}

/**************************************************
* �������ܣ�	����������ȡ����ֵ
**************************************************/
uint32_t Abs_int(int nValue)
{
	if(nValue < 0)	return (-nValue);
	else						return nValue;
}

/**************************************************************************
�������ܣ�485��ַ���ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Pdu_Init(void)
{
	enum enum_CarType CarType = CARMODE;	//С������
	enum enum_motor_type motor_type[MAX_MOTOR_NUMBER] = {0};	//�������
	enum enum_sport_mode sport_mode[MAX_MOTOR_NUMBER] = {0};	//����˶�ģʽ
	uint16_t CAN_id[MAX_MOTOR_NUMBER] = {0};					//���canid
	uint16_t reduction_ratio[MAX_MOTOR_NUMBER] = {0};			//������ٱ�
	uint16_t direction[MAX_MOTOR_NUMBER] = {0};					//�������
	uint16_t acceleration_time[MAX_MOTOR_NUMBER] = {0};			//����ʱ�䣨ZLAC��
	uint16_t deceleration_time[MAX_MOTOR_NUMBER] = {0};			//����ʱ�䣨ZLAC��
	uint16_t profile_rpm[MAX_MOTOR_NUMBER] = {0};				//����ٶȣ�WANZE��
	uint16_t profile_acce[MAX_MOTOR_NUMBER] = {0};				//���ٶȣ�WANZE��
	uint16_t profile_jerk[MAX_MOTOR_NUMBER] = {0};				//�Ӽ��ٶȣ�WANZE��

	struct_RobotBasePara AkmCarBasePara = {
		.car_type = Akm_Car,					
		.car_product_number = AKM_PRODUCT_NUM,	
		.car_version = AKM_VERSION,				
		.car_length = 11460,					
		.car_width = 7220,						
		.car_height = 3940,						
		.car_wheelbase = 6500,					//���
		.car_tread = 5800,					    //�־�
		.car_ground_clearance = 890,			//���̸߶�
		.wheel_radius = 1650,					//���ְ뾶
		.gross_max = 80,						//������
		.rated_load = 70,						//��غ�
		.motor_number = 3,						//�������
		.driving_method = rear_drive,			//������ʽ
	};  
	struct_RobotBasePara DiffCarPara = {
		.car_type = Diff_Car,
		.car_product_number = DIFF_PRODUCT_NUM,
		.car_version = DIFF_VERSION,
		.car_length = NULL,
		.car_width = NULL,
		.car_height = NULL,
		.car_wheelbase = 0,
		.car_tread = 3500,	
		.car_ground_clearance = NULL,
		.wheel_radius = 695,
		.gross_max = NULL,
		.rated_load = 80,
		.motor_number = 1,//ʵ��Ϊ2��������ͳһ���㣬д1��Ϊ�˺�ʵ��pdo��������һ�϶�
		.driving_method = full_drive,  
	};
	struct_RobotBasePara FourWheelCarPara = {
		.car_type = FourWheel_Car,
		.car_product_number = FOURWHEEL_PRODUCT_NUM,
		.car_version = FOURWHEEL_VERSION,
		.car_length = 10600,
		.car_width = 8300,
		.car_height = 4000,
		.car_wheelbase = 6600,
		.car_tread = 7050,
		.car_ground_clearance = 1300,
		.wheel_radius = 1650,
		.gross_max = 75,
		.rated_load = 100,
		.motor_number = 4,
		.driving_method = full_drive,
	};
	struct_RobotBasePara RCCarBasePara = {
		.car_type = RC_Car,
		.car_product_number = RC_PRODUCT_NUM,
		.car_version = RC_VERSION,
		.car_length = NULL,
		.car_width = NULL,
		.car_height = NULL,
		.car_wheelbase = NULL,
		.car_tread = NULL,
		.car_ground_clearance = NULL,
		.wheel_radius = NULL,
		.gross_max = NULL,
		.rated_load = NULL,
		.motor_number = 3,
		.driving_method = rear_drive,
	};
	struct_RobotBasePara DefaultBasePara = {0};
	struct_RobotBasePara* RobotBasePara;

	switch (CarType){
		case Akm_Car:
			RobotBasePara = &AkmCarBasePara;
			motor_type[0] = motor_type[1] = motor_type[2] = servo_zlac;
			sport_mode[0] = position_mode; sport_mode[1] = sport_mode[2] = speed_mode;
			CAN_id[0] = 4; CAN_id[1] = 5; CAN_id[2] = 6;
			reduction_ratio[0] = reduction_ratio[1] = reduction_ratio[2] = 30;
			direction[0] = direction[1] = direction[2] = car_direct_forward;
			acceleration_time[0] = 50; acceleration_time[1] = acceleration_time[2] = 100;
			deceleration_time[0] = 50; deceleration_time[1] = deceleration_time[2] = 100;
			profile_rpm[0] = profile_rpm[1] = profile_rpm[2] = 60000;
			profile_acce[0] = profile_acce[1] = profile_acce[2] = 2512;	
			profile_jerk[0]	= profile_jerk[1] = profile_jerk[2] = 10000;	
			break;
		case Diff_Car:
			RobotBasePara = &DiffCarPara;
			motor_type[0] = servo_zlacd;
			sport_mode[0] = speed_mode;
			CAN_id[0] = 1;
			reduction_ratio[0] = 1;
			direction[0] = car_direct_forward;
			acceleration_time[0] = 100;
			deceleration_time[0] = 100;
			profile_rpm[0] = 60000;
			profile_acce[0] = 2512;	
			profile_jerk[0]	= 10000;	
			break;
		case FourWheel_Car:
			RobotBasePara = &FourWheelCarPara;
			motor_type[0] = motor_type[1] = motor_type[2] = motor_type[3] = servo_wanze;
			sport_mode[0] = sport_mode[1] = sport_mode[2] = sport_mode[3] = speed_mode;
			CAN_id[0] = 1; CAN_id[1] = 5; CAN_id[2] = 2; CAN_id[3] = 6;
			reduction_ratio[0] = reduction_ratio[1] = reduction_ratio[2] = reduction_ratio[3] = 28;
			direction[0] = direction[1] = direction[2] = direction[3] = car_direct_forward;
			acceleration_time[0] = acceleration_time[1] = acceleration_time[2] = acceleration_time[3] = 100;
			deceleration_time[0] = deceleration_time[1] = deceleration_time[2] = deceleration_time[3] = 100;
			profile_rpm[0] = profile_rpm[1] = profile_rpm[2] = profile_rpm[3] = 60000;
			profile_acce[0] = profile_acce[1] = profile_acce[2] = profile_acce[3] = 2512;	
			profile_jerk[0]	= profile_jerk[1] = profile_jerk[2] = profile_jerk[3] = 10000;
			break;
		case RC_Car:
			RobotBasePara = &RCCarBasePara;
			/* code */
			break;			
		default:
			RobotBasePara = &DefaultBasePara;
			/* code */
			break;
	}
	rt_memcpy(pdu, RobotBasePara, sizeof(struct_RobotBasePara));
	int i;	
	if(pdu[car_type] == Diff_Car){
		pdu[TpdoGroupCount] = 1;	
		pdu[BatteryManufacturer] = lishen;
	}else{
	pdu[TpdoGroupCount] = 2;
		pdu[BatteryManufacturer] = batterydefault;
	}
	pdu[ro_motor_gap] = motor2_type - motor1_type;
	pdu[rw_motor_gap] = motor2_CAN_id - motor1_CAN_id;
	pdu[torque_cofficient] = TORQUE_COEFFICIENT_MIN;
	for (i = 0; i < pdu[motor_number]; i++){
		pdu[motor1_type + i * pdu[ro_motor_gap]] = motor_type[i];
		pdu[motor1_sport_mode + i * pdu[ro_motor_gap]] = sport_mode[i];
		pdu[motor1_CAN_id + i * pdu[rw_motor_gap]] = CAN_id[i];
		pdu[motor1_direction + i * pdu[rw_motor_gap]] = direction[i];
		pdu[motor1_reduction_ratio + i * pdu[rw_motor_gap]] = reduction_ratio[i];
		pdu[motor1_acceleration_time + i * pdu[rw_motor_gap]] = acceleration_time[i];
		pdu[motor1_deceleration_time + i * pdu[rw_motor_gap]] = deceleration_time[i];
		pdu[motor1_profile_rpm + i * pdu[rw_motor_gap]] = profile_rpm[i];
		pdu[motor1_profile_acce + i * pdu[rw_motor_gap]] = profile_acce[i];
		pdu[motor1_profile_jerk + i * pdu[rw_motor_gap]] = profile_jerk[i];
	}
	//pdu�������ݳ�ʼ��
	i = rc_min_value;
	pdu[i++] = RC_MIN_VALUE;
	pdu[i++] = RC_BASE_VALUE;
	pdu[i++] = RC_MAX_VALUE;
	pdu[i++] = RC_GEARS_DIFFERENCE;
	i = CAN_baud;
	pdu[i++] = can_baud_1000000;
	pdu[i++] = 1;
	pdu[i++] = usart_baud_2000000;
	pdu[i++] = control_mode_unknown;
	i = robot_acceleration;
	pdu[i++] = 5000;//�����˼��ٶ�ϵ��
	pdu[i++] = car_direct_forward;//������ǰ������
	pdu[i++] = car_direct_forward;//������ת�䷽��
	switch (pdu[car_type]){
		case Akm_Car:
			pdu[i++] = AKMCAR_MAXLINEARVELOCITY;//max_linear_speed
			pdu[i++] = AKMCAR_MAXYAWVELOCITY;	//max_yaw_speed
			pdu[i++] = MAXDEGREE;				//max_angle
			i = linear_low;
			pdu[i++] = (short)AKMCAR_SPEED_LOW;	//linear_low
			pdu[i++] = (short)AKMCAR_SPEED_MIDDLE;//linear_middle
			pdu[i++] = (short)AKMCAR_SPEED_HIGH;//linear_high
			pdu[i++] = (short)AKMCAR_YAW_LOW;	//angular_low
			pdu[i++] = (short)AKMCAR_YAW_MIDDLE;//angular_middle
			pdu[i++] = (short)AKMCAR_YAW_HIGH;	//angular_high
			break;
		case Diff_Car:
			pdu[i++] = DIFFCAR_MAXLINEARVELOCITY;
			pdu[i++] = DIFFCAR_MAXYAWVELOCITY;
			pdu[i++] = MAXDEGREE;		
			i = linear_low;
			pdu[i++] = (short)DIFFCAR_SPEED_LOW;
			pdu[i++] = (short)DIFFCAR_SPEED_MIDDLE;
			pdu[i++] = (short)DIFFCAR_SPEED_HIGH;
			pdu[i++] = (short)DIFFCAR_YAW_LOW;
			pdu[i++] = (short)DIFFCAR_YAW_MIDDLE;
			pdu[i++] = (short)DIFFCAR_YAW_HIGH;
			break;
		case FourWheel_Car:
			pdu[i++] = FOURWHEELCAR_MAXLINEARVELOCITY;
			pdu[i++] = FOURWHEELCAR_MAXYAWVELOCITY;
			pdu[i++] = MAXDEGREE;		
			i = linear_low;
			pdu[i++] = (short)FOURWHEELCAR_SPEED_LOW;
			pdu[i++] = (short)FOURWHEELCAR_SPEED_MIDDLE;
			pdu[i++] = (short)FOURWHEELCAR_SPEED_HIGH;
			pdu[i++] = (short)FOURWHEELCAR_YAW_LOW;
			pdu[i++] = (short)FOURWHEELCAR_YAW_MIDDLE;
			pdu[i++] = (short)FOURWHEELCAR_YAW_HIGH;	
			break;
		case TwoWheel_Car:
			pdu[i++] = TWOWHEELCAR_MAXLINEARVELOCITY;
			pdu[i++] = TWOWHEELCAR_MAXYAWVELOCITY;
			pdu[i++] = MAXDEGREE;		
			i = linear_low;
			pdu[i++] = (short)TWOWHEELCAR_SPEED_LOW;
			pdu[i++] = (short)TWOWHEELCAR_SPEED_MIDDLE;
			pdu[i++] = (short)TWOWHEELCAR_SPEED_HIGH;
			pdu[i++] = (short)TWOWHEELCAR_YAW_LOW;
			pdu[i++] = (short)TWOWHEELCAR_YAW_MIDDLE;
			pdu[i++] = (short)TWOWHEELCAR_YAW_HIGH;				
			break;
		case Tank_Car:
			pdu[i++] = TANKCAR_MAXLINEARVELOCITY;
			pdu[i++] = TANKCAR_MAXYAWVELOCITY;
			pdu[i++] = MAXDEGREE;		
			i = linear_low;
			pdu[i++] = (short)TANKCAR_SPEED_LOW;
			pdu[i++] = (short)TANKCAR_SPEED_MIDDLE;
			pdu[i++] = (short)TANKCAR_SPEED_HIGH;
			pdu[i++] = (short)TANKCAR_YAW_LOW;
			pdu[i++] = (short)TANKCAR_YAW_MIDDLE;
			pdu[i++] = (short)TANKCAR_YAW_HIGH;	
			break;
		default: 
			break;
	}

	pdu[virtually_rc_ch1_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch2_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch3_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch4_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch5_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch6_value] = RC_MIN_VALUE;
	pdu[virtually_rc_ch7_value] = RC_MIN_VALUE;
	pdu[virtually_rc_ch8_value] = RC_MIN_VALUE;
	pdu[virtually_rc_ch9_value] = RC_MIN_VALUE;
	pdu[virtually_rc_ch10_value] =RC_MIN_VALUE;
	pdu[virtually_rc_ch11_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch12_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch13_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch14_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch15_value] = RC_BASE_VALUE;
	pdu[virtually_rc_ch16_value] = RC_BASE_VALUE;

	pdu[Middle_battery_threshold] = 40;
	pdu[Low_battery_threshold] = 25;
	pdu[Power_board_version] = power_board_version_new;






}

// д�����пɶ���д����
void Modbus_Respond(MBModify* modify) 
{
	if (modify->is_modify == 0)return;
	if (pdu[para_save] == 3)
	{//< ��ǰ�޸Ĳ���д��flash
		pdu[para_save] = 0;
		switch (modify->modify_addr)
		{
		case motor1_direction:
			pdu[motor1_direction];
			break;
		}
	}

}


