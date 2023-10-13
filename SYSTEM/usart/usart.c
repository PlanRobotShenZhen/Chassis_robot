#include "usart.h"	
#include "mb.h"
 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif

uint8_t uart1_recv_data[USART1_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t uart1_send_data[USART1_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint32_t uart1_recv_flag = 0;        // 接收完成标志位
uint32_t uart1_recv_len = 0;         // 接收的数据长度
uint32_t uart1_send_len = 0;         // 发送的数据长度
uint32_t uart1_send_flag = 0;        // 发送完成标志位

/*****加入以下代码,支持printf函数,而不需要选择use MicroLIB****/	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch,FILE *p)  //函数默认的，在使用printf函数时自动调用
{
	USART_SendData(USART1,(u8)ch);	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

#endif 
/************************************************************/

/**
 *  @brief  MA1 通道4, UART1_TX 中断控制器配置
 *  @param  无
 *  @retval 无
 *  @note   中断优先级分组全工程只配置一次，在 main 函数最开始进行配置
 *  @note   中断处理函数在 CMSIS/stm32f10x_it.c 中进行处理
 */
void DMA1_Channel4_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	// 配置串口1的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // 在 stm32f10x.h 中找 IRQn_Type 枚举
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  串口1中断控制器配置
 *  @param  无
 *  @retval 无
 *  @note   中断优先级分组全工程只配置一次，在 main 函数最开始进行配置
 *  @note   中断处理函数在 CMSIS/stm32f10x_it.c 中进行处理
 */
void uart1_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	// 配置串口1的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;          // 在 stm32f10x.h 中找 IRQn_Type 枚举
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  串口1 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART1_TX -> DMA1 Channel4; UART1_RX -> DMA1 Channel5
 */
void uart1_dma_config(void)
{
	DMA_InitTypeDef DMA_InitStruct;

	DMA_DeInit(DMA1_Channel4);  // DMA1 通道4, UART1_TX
	DMA_DeInit(DMA1_Channel5);  // DMA1 通道5, UART1_RX

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // 使能DMA1的时钟

	// 配置 DMA1 通道4, UART1_TX
	DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_send_data;  // 内存地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStruct);

	// 配置 DMA1 通道5, UART1_RX
	DMA_InitStruct.DMA_PeripheralBaseAddr = (USART1_BASE + 0x04);   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)uart1_recv_data;  // 内存地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;                 // 外设到内存
	DMA_InitStruct.DMA_BufferSize = USART1_RX_MAXBUFF;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStruct);

	USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);// 使能DMA串口发送和接受请求

	DMA1_Channel4_nvic_config();

	// 配置 DMA1 通道4, UART1_TX 传输完成中断
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Channel5, ENABLE);     // 开启接收
	DMA_Cmd(DMA1_Channel4, DISABLE);    // 禁止发送
}


/**
 *  @brief  串口1初始化
 *  @param  波特率
 *  @retval 无
 *  @note   UART1_TX -> PA9; UART1_RX -> PA10
 */
void USART1_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;

	GPIO_DeInit(GPIOA);     // 恢复 GPIOA 的寄存器到默认值
	USART_DeInit(USART1);   // 恢复 UART1 的寄存器到默认值

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 使能 GPIOA 的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   // 使能 UART1 的时钟

	// 配置发送引脚 PA9 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 配置接收引脚 PA10 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// 配置串口参数 收发一体、8n1、无流控
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_BaudRate = baud;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStruct);

	uart1_nvic_config();    // 中断控制器配置
	uart1_dma_config();     // DMA配置

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // 使能空闲中断

	USART_Cmd(USART1, ENABLE);  // 使能串口1
}


#if EN_USART1_RX   //如果使能了接收
/*******************************************************************************
* 函 数 名         : USART1_IRQHandler
* 函数功能		   : USART1中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		USART1->SR; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USART1->DR; // 清除空闲中断
		DMA_Cmd(DMA1_Channel5, DISABLE);//< 暂停接收
		uart1_recv_flag = 1;                // 接收标志置1
		// 统计收到的数据的长度
		uart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(DMA1_Channel5);
	}
} 	

// DMA1 通道4, UART1_TX 传输完成中断
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC4) != RESET) // DMA1 通道4, UART1_TX 传输完成
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);     // 清除中断

		uart1_send_flag = 1;
		DMA_Cmd(DMA1_Channel4, DISABLE);        // 关闭 DMA1 通道4, UART1_TX


		//memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // 清空接收缓冲区
		DMA_Cmd(DMA1_Channel5, DISABLE);    // DMA1 通道5, UART1_RX
		DMA_SetCurrDataCounter(DMA1_Channel5, USART1_RX_MAXBUFF);
		DMA_Cmd(DMA1_Channel5, ENABLE);     // DMA1 通道5, UART1_RX
	}
}
void modbus_task_init(void)
{
	UCHAR mySlaveAddress = 0x01;//< 从机地址
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
}
void modbus_task(void* pvParameters)
{
	TickType_t lastWakeTime = (TickType_t)getSysTickCnt();
	/* Enable the Modbus Protocol Stack. */
	eMBEnable();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, 1);//此任务以1000Hz的频率运行
		if (uart1_recv_flag == 1)
		{
			uart1_recv_flag = 0;
			pxMBFrameCBByteReceived();
		}
		eMBPoll();
		if (uart1_send_flag == 2)
		{//< 回复帧
			uart1_send_flag = 0;
			DMA_Cmd(DMA1_Channel4, DISABLE);    // 关闭 DMA1 通道4, UART1_TX
			DMA_SetCurrDataCounter(DMA1_Channel4, uart1_send_len);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
			DMA_Cmd(DMA1_Channel4, ENABLE);    // 开启 DMA1 通道4, UART1_TX
		}
	}
}

  
#endif	

