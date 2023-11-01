#include "usartx.h"
#include "user_can.h"
#include "485_address.h"


/*--------����λ��ͨ�Ÿ�ʽ����usart3-----------*/
SEND_DATA Send_Data;//�������ݵĽṹ��
RECEIVE_DATA Receive_Data;//�������ݵĽṹ��

#define USART3_RX_MAXBUFF 25
#define USART3_TX_MAXBUFF 25
uint8_t uart3_recv_data[USART3_RX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t uart3_send_data[USART3_TX_MAXBUFF] = { 0 };  // �������ݻ�����
uint8_t uart3_recv_flag = 0;        // ������ɱ�־λ
uint8_t uart3_recv_len = 0;         // ���յ����ݳ���
uint8_t uart3_send_len = 0;         // ���͵����ݳ���
uint8_t uart3_send_flag = 1;        // ������ɱ�־λ
/*--------��ģ���ݽ������ȫ�ֱ����Ķ���-----------*/
SBUS_CH_Struct tagSBUS_CH;
unsigned char Uart5_Buffer[Max_BUFF_Len] = {0};  
unsigned int  ucRcvCount = 0; 
uint8_t ucRcvReady = 0;
uint8_t is_connect = 0;
int g_nVelocity = 0;       //ǰ������ٶ�                   
int g_nDirector = 0;       //ת��

int g_nLastVelocity = 0;          //��ŵ�ǰ���ٶȺ���һ�̵��ٶ�
int g_nCurrentVelocity = 0;

unsigned char g_ucLightOnFlag = 0; //���ƿ�����־
float g_fltRecv_Vel_X = 0.0;                        // ���ڽ��յ����ٶ�����
float g_fltRecv_Vel_Y = 0.0;
float g_fltRecv_Vel_Z = 0.0;


/**************************************************************************
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low)
{
	short transition; //����ת�����м����
	transition = ((High << 8) + Low); //����8λ�͵�8λ���ϳ�һ��16λ��short������
	return transition / 1000 + (transition % 1000) * 0.001;    //���Ͷ˽����ݷ���ǰ����һ��*1000�ĵ�λ���㣬����������ݺ���Ҫ��ԭ��λ
}
/**************************************************************************
�������ܣ������շ��վ�������,����3�������ݵ���λ�����жϽ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//��������20Hz��Ƶ������
			if (uart3_recv_flag)
			{
				uart3_recv_flag = 0;
				if (uart3_recv_len == 11)
				{
					if (Receive_Data.buffer[10] == FRAME_TAIL) //��֤���ݰ���β��У����Ϣ
					{
						if (Receive_Data.buffer[9] == Check_Sum(9, 0))	 //����У��λ���㣬ģʽ0�Ƿ�������У��
						{
							g_fltRecv_Vel_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
							g_fltRecv_Vel_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
							g_fltRecv_Vel_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
						}
					}

				}

			}

			if (uart3_send_flag == 1)
			{//< �ظ�֡
				uart3_send_flag = 0;
				Data_transition(); //��Ҫ���з��͵����ݽ��и�ֵ
				USART3_SEND();     //����3(ROS)��������
			}
		}
}


/**************************************************
* �������ܣ�	����5��ʼ����������Ϊ��ģ�źŽ���
* ��    ����  unBound����ͨ�Ų����ʣ���Ϊ100k
* �� �� ֵ��  ��
**************************************************/
void Usart5_Init(unsigned int unBound)
{
	//GPIO�˿�����
    GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_UART5, ENABLE);	//ʹ��USART2��GPIOAʱ��
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD, ENABLE);
	
	//USART5_TX   GPIOC.12
    GPIO_InitStructure.Pin = GPIO_PIN_12; //PC12
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC.12
   
  //USART5_RX	  GPIOD.2��ʼ��
    GPIO_InitStructure.Pin = GPIO_PIN_2;//PD2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOD.2  

  //USART2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.BaudRate = unBound;//���ڲ�����
	USART_InitStructure.WordLength = USART_WL_9B;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.StopBits = USART_STPB_1;//һ��ֹͣλ
	USART_InitStructure.Parity = USART_PE_EVEN;//����żУ��λ
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;//��Ӳ������������
	USART_InitStructure.Mode = USART_MODE_RX;	//�շ�ģʽ

    USART_Init(UART5, &USART_InitStructure); //��ʼ������5
    USART_ConfigInt(UART5, USART_INT_RXDNE, ENABLE);//�������ڽ����ж�
	//USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
    USART_Enable(UART5, ENABLE);                    //ʹ�ܴ���5 
}

/**
 *  @brief  DMA1 ͨ��2, UART3_TX �жϿ���������
 *  @param  ��
 *  @retval ��
 *  @note   �ж����ȼ�����ȫ����ֻ����һ�Σ��� main �����ʼ��������
 *  @note   �жϴ������� CMSIS/stm32f10x_it.c �н��д���
 */
void DMA1_Channel2_nvic_config(void)
{
	NVIC_InitType NVIC_InitStruct;

	// ���ô���1���жϿ�����
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;   // �� stm32f10x.h ���� IRQn_Type ö��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  ����3 DMA��ʼ������
 *  @param  ��
 *  @retval ��
 *  @note    UART3_TX -> DMA1 Channel2; UART3_RX -> DMA1 Channel3
 */
void uart3_dma_config(void)
{
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(DMA1_CH2);  // DMA1 ͨ��2, UART3_TX
	DMA_DeInit(DMA1_CH3);  // DMA1 ͨ��3, UART3_RX

	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1, ENABLE);  // ʹ��DMA1��ʱ��

	// ���� DMA1 ͨ��2, UART3_TX
	DMA_InitStruct.PeriphAddr = (USART3_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart3_send_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // �Ĵ���������Ϊ0ʱ������ͨ���Ƿ����������ᷢ���κ����ݴ���
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(DMA1_CH2, &DMA_InitStruct);

	// ���� DMA1 ͨ��3, UART3_RX
	DMA_InitStruct.PeriphAddr = (USART3_BASE + 0x04);   // ���ݼĴ���(USART_DR) ��ַƫ�ƣ�0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart3_recv_data;  // �ڴ��ַ
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // ���赽�ڴ�
	DMA_InitStruct.BufSize = USART3_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(DMA1_CH3, &DMA_InitStruct);

	USART_EnableDMA(USART3, USART_DMAREQ_TX | USART_DMAREQ_RX, ENABLE);// ʹ��DMA���ڷ��ͺͽ�������

	DMA1_Channel2_nvic_config();

	// ���� DMA1 ͨ��2, UART3_TX ��������ж�
	DMA_ConfigInt(DMA1_CH2, DMA_INT_TXC, ENABLE);

	DMA_EnableChannel(DMA1_CH2, ENABLE);     // ��������
	DMA_EnableChannel(DMA1_CH3, DISABLE);    // ��ֹ����
}

/**************************************************************************
�������ܣ�����3��ʼ������Ϊ����λ��ͨ��
��ڲ�����boundΪͨ�Ų�����
����PCB���еĵڶ���USB�ӿ�
�� �� ֵ����
**************************************************************************/
void Usart3_init(u32 bound)
{
	//GPIO�˿�����
    GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);	//ʹ��GPIOʱ��
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);	//ʹ��USART3ʱ��
	GPIO_ConfigPinRemap(GPIO_PART_RMP_USART3, ENABLE);//������ӳ��
	//USART_TX  
    GPIO_InitStructure.Pin = GPIO_PIN_10; //C10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);   
    //USART_RX	  
    GPIO_InitStructure.Pin = GPIO_PIN_11;//PC11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    //UsartNVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8 ;//��ռ���ȼ����ж����ȼ����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    //USART ��ʼ������ 
	USART_InitStructure.BaudRate = bound;//���ڲ�����
	USART_InitStructure.WordLength = USART_WL_8B;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.StopBits = USART_STPB_1;//һ��ֹͣλ
	USART_InitStructure.Parity = USART_PE_NO;//����żУ��λ
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;//��Ӳ������������
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);     //��ʼ������3

    uart3_dma_config();     // DMA����

    USART_ConfigInt(USART3, USART_INT_IDLEF, ENABLE);//�������ڽ����ж�
    USART_Enable(USART3, ENABLE);                    //ʹ�ܴ���3 
}


void SBUSDataRefresh(uint16_t* pdu)
{
	memcpy(&pdu[remote_ch1_value], &tagSBUS_CH, sizeof(SBUS_CH_Struct));
	is_connect++;
	if (is_connect > 5)
	{
		is_connect = 0;
		tagSBUS_CH.ucConnectState = 0;
	}
}

/**************************************************
* �������ܣ�	��sbus�ź�ת��Ϊͨ��ֵ
* ��    ����  ucBufΪ���յ��Ĵ�������
* �� �� ֵ��  0����ɹ���1����ʧ��
**************************************************/
unsigned char Update_sbus(unsigned char* ucBuf)
{
	if(ucBuf[23] == 0)
	{
		is_connect = 0;
		tagSBUS_CH.ucConnectState = 1;
		tagSBUS_CH.CH1 = ((int16_t)ucBuf[ 1] >> 0  | ((int16_t)ucBuf[ 2] << 8 )) & 0x07FF;
		tagSBUS_CH.CH2 = ((int16_t)ucBuf[ 2] >> 3  | ((int16_t)ucBuf[ 3] << 5 )) & 0x07FF;
		tagSBUS_CH.CH3 = ((int16_t)ucBuf[ 3] >> 6  | ((int16_t)ucBuf[ 4] << 2 ) | (int16_t)ucBuf[ 5] << 10 ) & 0x07FF;
		tagSBUS_CH.CH4 = ((int16_t)ucBuf[ 5] >> 1  | ((int16_t)ucBuf[ 6] << 7 )) & 0x07FF;
		tagSBUS_CH.CH5 = ((int16_t)ucBuf[ 6] >> 4  | ((int16_t)ucBuf[ 7] << 4 )) & 0x07FF;
		tagSBUS_CH.CH6 = ((int16_t)ucBuf[ 7] >> 7  | ((int16_t)ucBuf[ 8] << 1 ) | (int16_t)ucBuf[9] << 9 ) & 0x07FF;
		tagSBUS_CH.CH7 = ((int16_t)ucBuf[ 9] >> 2  | ((int16_t)ucBuf[10] << 6 )) & 0x07FF;
		tagSBUS_CH.CH8 = ((int16_t)ucBuf[10] >> 5  | ((int16_t)ucBuf[11] << 3 )) & 0x07FF;
		tagSBUS_CH.CH9 = ((int16_t)ucBuf[12] << 0  | ((int16_t)ucBuf[13] << 8 )) & 0x07FF;
		tagSBUS_CH.CH10 = ((int16_t)ucBuf[13] >> 3 | ((int16_t)ucBuf[14] << 5 )) & 0x07FF;
		tagSBUS_CH.CH11 = ((int16_t)ucBuf[14] >> 6 | ((int16_t)ucBuf[15] << 2 ) | (int16_t)ucBuf[16] << 10 ) & 0x07FF;
		tagSBUS_CH.CH12 = ((int16_t)ucBuf[16] >> 1 | ((int16_t)ucBuf[17] << 7 )) & 0x07FF;
		tagSBUS_CH.CH13 = ((int16_t)ucBuf[17] >> 4 | ((int16_t)ucBuf[18] << 4 )) & 0x07FF;
		tagSBUS_CH.CH14 = ((int16_t)ucBuf[18] >> 7 | ((int16_t)ucBuf[19] << 1 ) | (int16_t)ucBuf[20] << 9 ) & 0x07FF;
		tagSBUS_CH.CH15 = ((int16_t)ucBuf[20] >> 2 | ((int16_t)ucBuf[21] << 6 )) & 0x07FF;
		tagSBUS_CH.CH16 = ((int16_t)ucBuf[21] >> 5 | ((int16_t)ucBuf[22] << 3 )) & 0x07FF;
		
		return 1;
	}
	return 0;
	
}


/**************************************************
* �������ܣ�	�źŽ����жϴ�����
* ��ڲ�����  ��
* �� �� ֵ��  ��
**************************************************/
void UART5_IRQHandler() 
{	
	if(USART_GetIntStatus(UART5,USART_INT_RXDNE) != RESET) //�жϲ��� 
	{
		g_eControl_Mode = CONTROL_MODE_REMOTE; //��ģң�ط�ʽѡ��
		USART_ClrIntPendingBit(UART5,USART_INT_RXDNE); //����жϱ�־			 
		Uart5_Buffer[ucRcvCount++] = USART_ReceiveData(UART5);     //���մ���1���ݵ�buff������
		if (Uart5_Buffer[0] == 0x0f)
		{
			if (ucRcvCount > 23)
			{
				ucRcvReady = 1;
				ucRcvCount = 0;
			}
		}
		else
		{
			ucRcvCount = 0;                                   //����������Ҫ�����ݻ��ߴﵽ����������ʼ���½���
		}
	}
}


// DMA1 ͨ��2, UART3_TX ��������ж�
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC2, DMA1) != RESET) // DMA1 ͨ��2, UART3_TX �������
	{
		DMA_ClrIntPendingBit(DMA1_INT_TXC2, DMA1);     // ����ж�

		uart3_send_flag = 1;
		DMA_EnableChannel(DMA1_CH2, DISABLE);        // �ر� DMA1 ͨ��4, UART1_TX
		//memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // ��ս��ջ�����
		DMA_EnableChannel(DMA1_CH3, DISABLE);    // DMA1 ͨ��3, UART3_RX
		DMA_SetCurrDataCounter(DMA1_CH3, USART3_RX_MAXBUFF);
		DMA_EnableChannel(DMA1_CH3, ENABLE);     // DMA1 ͨ��3, UART3_RX
	}
}
/**************************************************************************
�������ܣ�����3�����ж�,������λ�����͹���������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void USART3_IRQHandler(void)
{	
#if(1)
	if (USART_GetIntStatus(USART3, USART_INT_IDLEF) != RESET)
	{
		USART3->STS; // ��������ж�, ��������������λ(�ȶ�USART_SR��Ȼ���USART_DR)
		USART3->DAT; // ��������ж�
		uart3_recv_flag = 1;                // ���ձ�־��1
		g_ucRos_Flag = 1;                   // ��ģ������ʱ�򣬸����������ֵΪ0
		if (g_ucRemote_Flag == 0 && g_ucRos_Flag == 1)
		{
			g_eControl_Mode = CONTROL_MODE_ROS;   // ΪROS��λ������
		}
		// ͳ���յ������ݵĳ���
		uart3_recv_len = USART3_RX_MAXBUFF - DMA_GetCurrDataCounter(DMA1_CH3);
		memcpy(Receive_Data.buffer, uart3_recv_data, uart3_recv_len);
		DMA_EnableChannel(DMA1_CH3, DISABLE);    // DMA1 ͨ��3, UART3_RX
		DMA_SetCurrDataCounter(DMA1_CH3, USART3_RX_MAXBUFF);
		DMA_EnableChannel(DMA1_CH3, ENABLE);     // DMA1 ͨ��3, UART3_RX
	}
#else
	static unsigned char ucCount=0;
	unsigned char ucUsart_Receive;

	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�ж��Ƿ���յ�����
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //����жϱ�־
		
		g_ucRos_Flag = 1;                        // ��ģ������ʱ�򣬸����������ֵΪ0
		if(g_ucRemote_Flag == 0 && g_ucRos_Flag == 1)
		{
			g_eControl_Mode = CONTROL_MODE_ROS;   // ΪROS��λ������
		}
		
		ucUsart_Receive = USART_ReceiveData(USART3);//��ȡ����
    Receive_Data.buffer[ucCount] = ucUsart_Receive;
		if(ucUsart_Receive == FRAME_HEADER || ucCount > 0) 
		{
			ucCount++;
		} 
		else 
		{
			ucCount = 0;
		}
		if (ucCount == 11)	//��֤���ݰ��ĳ���
		{   
			ucCount=0;//���¿�ʼ����
			if(Receive_Data.buffer[10] == FRAME_TAIL) //��֤���ݰ���β��У����Ϣ
			{
				if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 //����У��λ���㣬ģʽ0�Ƿ�������У��
				{			
//					Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
//					Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
//					Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
					g_fltRecv_Vel_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
					g_fltRecv_Vel_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
					g_fltRecv_Vel_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				}
			}
		}
	} 
#endif
}

/**************************************************************************
�������ܣ�����3(ROS)��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_SEND(void)
{
	uart3_send_len = 24;
	memcpy(uart3_send_data, Send_Data.buffer, uart3_send_len);
	DMA_EnableChannel(DMA1_CH2, DISABLE);    // �ر� DMA1 ͨ��2, UART3_TX
	DMA_SetCurrDataCounter(DMA1_CH2, uart3_send_len);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
	DMA_EnableChannel(DMA1_CH2, ENABLE);    // ���� DMA1 ͨ��2, UART3_TX	
}


/**************************************************************************
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void Usart3_send(unsigned char data)
{
	USART3->DAT = data;
	while((USART3->STS&0x40) == 0);	
}



/**************************************************************************
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void Data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL; //֡β
	
	switch(g_emCarMode)
	{	
		case Mec_Car:      //�����ķ��С�� 
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/4)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.fltFeedBack_Velocity - MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity - MOTOR_D.fltFeedBack_Velocity)/4)*1000; //С��y���ٶ�
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.fltFeedBack_Velocity - MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/4/(Axle_spacing+Wheel_spacing))*1000;//С��z���ٶ�            
		  break; 
		
    case Omni_Car: //ȫ����С��     
			Send_Data.Sensor_Str.X_speed = ((MOTOR_C.fltFeedBack_Velocity-MOTOR_B.fltFeedBack_Velocity)/2/X_PARAMETER)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.fltFeedBack_Velocity*2-MOTOR_B.fltFeedBack_Velocity-MOTOR_C.fltFeedBack_Velocity)/3)*1000; //С��y���ٶ�
	    Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.fltFeedBack_Velocity+MOTOR_B.fltFeedBack_Velocity+MOTOR_C.fltFeedBack_Velocity)/3/Omni_turn_radiaus)*1000;//С��z���ٶ�       
		  break; 
    
		case Akm_Car:   //������С��
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.fltFeedBack_Velocity+MOTOR_B.fltFeedBack_Velocity)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.fltFeedBack_Velocity-MOTOR_A.fltFeedBack_Velocity)/Wheel_spacing)*1000;//С��z���ٶ�
		  break; 
		
		case Diff_Car:  //���ֲ���С��
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity)/Wheel_spacing)*1000;//С��z���ٶ�
			break; 
		
		case FourWheel_Car: //������ 
      Send_Data.Sensor_Str.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity 
									+MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/4)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = 0;
		  /*Z�᷽���Ϸ��͵���һ�����ٶ���Ϣ��wr = v*/
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity + 
		MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/2/(Axle_spacing+Wheel_spacing))*1000;//С��z���ٶ�
		 break; 
		
		case Tank_Car:   //�Ĵ���
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity)/(Wheel_spacing)*1000);//С��z���ٶ�
			break; 
		default:
			break;
	}
	//���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2];
	
	//���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //���ٶȼ�X��ת����ROS����Y��
	if(Flag_Stop==0) Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];  //����������λʹ��״̬����ô��������Z����ٶ�
	else             Send_Data.Sensor_Str.Gyroscope.Z_data=0;       //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0
	
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; //��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //֡ͷ(�̶�ֵ)
  Send_Data.buffer[1]=Flag_Stop;//���״̬
	
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; //С��x���ٶ�
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    //С��x���ٶ�
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  //С��y���ٶ�
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     //С��y���ٶ�
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; //С��z���ٶ�
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    //С��z���ٶ�
	
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; //���ٶȼ�������ٶ�
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;    //���ٶȼ�������ٶ�
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8; //���ٶȼ�������ٶ�
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data; //���ٶȼ�������ٶ�
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; //��ص�ѹ
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; //��ص�ѹ

	Send_Data.buffer[22]=Check_Sum(22,1); //����У��λ���㣬ģʽ1�Ƿ�������У��
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail;//֡β���̶�ֵ��
}


/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����23λΪУ��λ�����������1-22�����������һ������Ϊ���ͻ��ǽ���У��λ
�� �� ֵ������λ
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum = 0,k;
	unsigned char* buff = Mode == 0? &Receive_Data.buffer[0]: &Send_Data.buffer[0];
	//Mode == 0�������ݵ�У��,Mode == 1�������ݵ�У��
	for (k = 0;k < Count_Number; k++)
	{
		check_sum = check_sum ^ buff[k];
	}	
	return check_sum;
}



/**************************************************
* �������ܣ�	��ȡ���趨���ٶȺͷ���ֵ
**************************************************/
void SetReal_Velocity(uint16_t* pdu)
{
	int nTemp = 0;  //��ʱ����
	int nSwitchSpeed = 0;
	if (ucRcvReady)
	{
		Update_sbus(Uart5_Buffer);
		ucRcvReady = 0;
	}
	if (g_eControl_Mode == CONTROL_MODE_UART)
	{
		SBUS_CH_Struct* uart_sbus = (SBUS_CH_Struct*) & pdu[130];
		memcpy(&tagSBUS_CH, uart_sbus, sizeof(SBUS_CH_Struct));
	}
	SBUSDataRefresh(pdu);
	//�ж�SWA�Ƿ��
	nTemp = tagSBUS_CH.CH7;
	if((Abs_int(nTemp - rc_ptr->turn_off_remote) < 10))
	{
		g_ucRemote_Flag = 0;                   //��ģ�رձ�־λ		
		//g_eControl_Mode = CONTROL_MODE_REMOTE; //��ģң�ط�ʽѡ��
		//���ش��ڹر�״̬
		g_nVelocity = 0;
		return;
	}
	 
	else if(Abs_int(nTemp - rc_ptr->turn_on_remote) < 10)
	{
		g_ucRemote_Flag = 1;                   // ��ģ������־λ
		g_ucRos_Flag = 0;                      // ��ģ�������������Ȩ��
		//g_eControl_Mode = CONTROL_MODE_REMOTE; //��ģң�ط�ʽѡ��
		//���ش��ڴ�״̬
		nTemp = tagSBUS_CH.CH6;
		
		//��ʱ�ж���ʹ�õ͡��С����ٶȵ�λ
		if(Abs_int(nTemp - rc_ptr->speed_level1) < 10)
		{
			//�������ת����1������ٷ�ת����-1�����෵�صĽ��ֻ��������ָʾ����������̫С��Ϊ0
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			//���ٵ�
			if(nSwitchSpeed > 0)
			{
				//�����ﵽ����ٶ�
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_low;
				else
					g_nVelocity = rc_ptr->speed_low * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;//784=���ֵ-��׼ֵ
			}
			else if(nSwitchSpeed < 0)
			{
				//��ת�ﵽ����ٶ�
				if(nSwitchSpeed == -1)
					g_nVelocity = -rc_ptr->speed_low;
				else
					g_nVelocity = rc_ptr->speed_low * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else
			{
				g_nVelocity = 0;
			}
		}
		else if(Abs_int(nTemp - rc_ptr->speed_level2) < 10)
		{
			//���ٵ�
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			if(nSwitchSpeed > 0)
			{
				//�����ﵽ����ٶ�
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_middle;
				else
					g_nVelocity = rc_ptr->speed_middle * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else if(nSwitchSpeed < 0)
			{
				//��ת�ﵽ����ٶ�
				if(nSwitchSpeed == -1)
					g_nVelocity = -rc_ptr->speed_middle;
				else
					g_nVelocity = rc_ptr->speed_middle * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else
			{
				g_nVelocity = 0;
			}
		}
		else if(Abs_int(nTemp - rc_ptr->speed_level3) < 10)
		{
			//���ٵ�
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			if(nSwitchSpeed > 0)
			{
				//�����ﵽ����ٶ�
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_high;
				else
					g_nVelocity = rc_ptr->speed_high * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else if(nSwitchSpeed < 0)
			{
				//��ת�ﵽ����ٶ�
				if(nSwitchSpeed == -1)
					g_nVelocity = -rc_ptr->speed_high;
				else
					g_nVelocity = rc_ptr->speed_high * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else
			{
				g_nVelocity = 0;
			}
		}
	}
	
	// ת�ǻ�ȡ
	Set_Director();
	// ���ƿ���
	Car_Light_Control();
}



/**************************************************
* �������ܣ�	����������ȡ����ֵ
**************************************************/
int Abs_int(int nValue)
{
	if(nValue < 0)
	{
		return (-nValue);
	}
	else
	{
		return nValue;
	}
}



/**************************************************
* �������ܣ�	��������ת���Ƿ�ת����
* �� �� ֵ��  ���������ֱ��������ת
**************************************************/
int Target_Velocity_get(unsigned short usValue)
{
	int nValue = 0;
	
	//�Ƚ����ٶ��޷��趨������ȡ�ٶ�
	if(usValue > rc_ptr->limit_max_val)
	{
		nValue = 1;
	}
	else if(usValue < rc_ptr->limit_min_val)
	{
		nValue = -1;
	}
	else
	{
		nValue = ((int)usValue - rc_ptr->vel_base_value);
		if(nValue < 10 && nValue > -10)
		{
			nValue = 0;
		}
		
	}
	return nValue;
}


/**************************************************
* �������ܣ�	��������ת���Ƿ�ת����
* �� �� ֵ��  ���������ֱ��������ת
**************************************************/
int Target_Direct_get(unsigned short usValue)
{
	int nValue = 0;
	
	//�Ƚ��нǶȵ��޷��趨������ȡ�Ƕ�
	if(usValue > rc_ptr->limit_max_val)
	{
		nValue = 1;
	}
	else if(usValue < rc_ptr->limit_min_val)
	{
		nValue = -1;
	}
	else
	{
		nValue = ((int)usValue - rc_ptr->dir_base_value);
		if(nValue < 10 && nValue > -10)
		{
			nValue = 0;
		}
		
	}
	return nValue;
}


/**************************************************
* �������ܣ�	��Z��ת��Ľ��ٶȻ�ȡ
* �� �� ֵ��  ��
**************************************************/
void Set_Director()
{
	
	int nTemp = 0;  //��ʱ����
	int nSwitchDirect = 0;  //ת�䷽���ѡ��
	
	//�ж�SWA�Ƿ��
	nTemp = tagSBUS_CH.CH7;
	if(Abs_int(nTemp - rc_ptr->turn_off_remote) < 10)
	{
		//���ش��ڹر�״̬
		g_nDirector = 0;
		return;
	}
	
	else if(Abs_int(nTemp - rc_ptr->turn_on_remote) < 10)
	{
		//���ش��ڴ�״̬
		nTemp = tagSBUS_CH.CH6;
		
		//��ʱ�ж���ʹ�õ͡��С����ٶȵ�λ
		if(Abs_int(nTemp - rc_ptr->speed_level1) < 10)
		{
			//���ٵ�
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//�Ҵ������
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_low;
				else
					g_nDirector = rc_ptr->speed_dir_low * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//��ת�����
				if(nSwitchDirect == -1)
					g_nDirector = -rc_ptr->speed_dir_low;
				else
					g_nDirector = rc_ptr->speed_dir_low * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else
			{
				g_nDirector = 0;
			}
		}
		else if(Abs_int(nTemp - rc_ptr->speed_level2) < 10)
		{
			//���ٵ�
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//�Ҵ������
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_middle;
				else
					g_nDirector = rc_ptr->speed_dir_middle * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//��ת�����
				if(nSwitchDirect == -1)
					g_nDirector = -rc_ptr->speed_dir_middle;
				else
					g_nDirector = rc_ptr->speed_dir_middle * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else
			{
				g_nDirector = 0;
			}
		}
		else if(Abs_int(nTemp - rc_ptr->speed_level3) < 10)
		{
			//���ٵ�
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//�Ҵ������
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_high;
				else
					g_nDirector = rc_ptr->speed_dir_high * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//��ת�����
				if(nSwitchDirect == -1)
					g_nDirector = -rc_ptr->speed_dir_high;
				else
					g_nDirector = rc_ptr->speed_dir_high * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else
			{
				g_nDirector = 0;
			}
		}
		else
		{
			g_nDirector = 0;
		}
	}
}



/****************************************************************
* �������ܣ��������ݽ���������ʹ��VRA�����ƣ���ȡͨ��10����ȡ����
* ����ֵ��
****************************************************************/
void Car_Light_Control(void)
{
	unsigned short usTemp = 0;
	
	usTemp = tagSBUS_CH.CH10;
	
	// �߼��ж�ģ��
	if(Abs_int(usTemp - rc_ptr->light_base) > 100)
	{
		// ��ʱ����û�д��������ƹ�ı�־

		if((usTemp - rc_ptr->light_base) > 0)
		{
			// �����ƹ�
			g_ucLightOnFlag = 1;
		}
		else
		{
			// �رյƹ�
			g_ucLightOnFlag = 0;
		}
	}
}


/*********************************************
* �������ܣ����ڵ��Դ�ӡimu��Ϣ

*********************************************/
void Printf_MPU9250_Data()
{
	//printf("/*********************imu message********************/ \r\n");
	//
	//printf("accel[0] is: %d\r\n",accel[0]);
	//printf("accel[1] is: %d\r\n",accel[1]);
	//printf("accel[2] is: %d\r\n",accel[2]);
	//
	//printf("gyro[0] is: %d\r\n",gyro[0]);
	//printf("gyro[1] is: %d\r\n",gyro[1]);
	//printf("gyro[2] is: %d\r\n",gyro[2]);
	//
	//printf("\r\n \r\n");
}
