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
/*--------上下位机通信格式数据usart3-----------*/
SEND_DATA Send_Data;//发送数据的结构体
unsigned char Receive_Data[RECEIVE_DATA_SIZE] = {0};//接收数据的结构体
unsigned char Receive_Null[RECEIVE_DATA_SIZE] = {0};
/*----------------------------usart1、3、5----------------------------*/
uint8_t usart1_send_data[USART1_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint8_t usart1_send_len = 0;         // 发送的数据长度
uint8_t usart1_send_flag = 0;        // 发送完成标志位

uint8_t usart1_recv_data[USART1_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t usart1_recv_flag = 0;        // 接收完成标志位
uint8_t usart1_recv_len = 0;         // 接收的数据长度

uint8_t usart3_send_data[USART3_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint8_t usart3_send_len = 0;         // 发送的数据长度
uint8_t usart3_send_flag = 1;        // 发送完成标志位

uint8_t usart3_recv_data[USART3_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t usart3_recv_flag = 0;        // 接收完成标志位
uint8_t usart3_recv_len = 0;         // 接收的数据长度

uint8_t uart4_send_data[UART4_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint8_t uart4_send_len = 0;         // 发送的数据长度
uint8_t uart4_send_flag = 0;        // 发送完成标志位

uint8_t uart4_recv_data[UART4_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t uart4_recv_flag = 0;        // 接收完成标志位
uint8_t uart4_recv_len = 0;         // 接收的数据长度

unsigned char Uart5_Buffer[Max_BUFF_Len] = { 0 }; 

bool ROS_RecvFlag = false;
bool Motor_Enable_Flag = false;
int ROS_Count = 0;
uint8_t CheckSum_sdo[1][8] = { 0 }; 

/*******************************************************************************USART1*******************************************************************************/
/*******************************************************************************
* 函 数 名: Usart1_Init
* 函数功能: USART1初始化函数
* 输    入: bound:波特率
* 输    出: 无
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	USART_ConfigInt(USARTOne, USART_INT_IDLEF, ENABLE);  // 使能空闲中断
	Usart1_Dma_Config();     // DMA配置
	USART_Enable(USARTOne, ENABLE);
}

/**
 *  @brief  MA1 通道4, UART1_TX 中断控制器配置
 *  @param  无
 *  @retval 无
 *  @note   中断优先级分组全工程只配置一次，在 main 函数最开始进行配置
 *  @note   中断处理函数在 CMSIS/stm32f10x_it.c 中进行处理
 */
void DMA1_Channel4_nvic_config(void)
{
	NVIC_InitType NVIC_InitStruct;

	// 配置串口1的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // 在 stm32f10x.h 中找 IRQn_Type 枚举
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);
}
/*******************************************************************************
 *  @brief  串口1 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART1_TX -> DMA1 Channel4; UART1_RX -> DMA1 Channel5
*******************************************************************************/
void Usart1_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(USARTOne_Tx_DMA_Channel);  // DMA1 通道4, UART1_TX
	DMA_DeInit(USARTOne_Rx_DMA_Channel);  // DMA1 通道5, UART1_RX

	RCC_EnableAHBPeriphClk(USARTOne_DMAx_CLK, ENABLE);

	// 配置 DMA1 通道4, UART1_TX
	DMA_InitStruct.PeriphAddr = USARTOne_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart1_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTOne_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART1_TX, DMA1, USARTOne_Tx_DMA_Channel, ENABLE);

	// 配置 DMA1 通道5, UART1_RX
	DMA_InitStruct.PeriphAddr = USARTOne_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart1_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
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

	// 配置串口1的中断控制器
	DMA1_Channel4_nvic_config();

	DMA_ConfigInt(USARTOne_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);// 配置 DMA1 通道4, UART1_TX 传输完成中断
	USART_EnableDMA(USARTOne, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);// 使能DMA串口发送和接受请求

	DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);     // 开启接收
	DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);    // 禁止发送
}

/*******************************************************************************
* 函 数 名         : USART1_IRQHandler
* 函数功能		   : USART1中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void USARTOne_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetIntStatus(USARTOne, USART_INT_IDLEF) != RESET){
		USARTOne->STS;
		USARTOne->DAT;
		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);     // 暂停接收
		usart1_recv_flag = 1;                // 接收标志置1
		usart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTOne_Rx_DMA_Channel);// 统计收到的数据的长度
	}
}

/*******************************************************************************
* 函数功能		   	 : USART1_TX 传输完成中断
* 输    入         : 无
* 说    明         : DMA1 通道4, UART1_TX 传输完成中断
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC4, DMA1)){ // DMA1 通道4, UART1_TX 传输完成
		DMA_ClrIntPendingBit(DMA1_INT_TXC4, DMA1);      // 清除中断
		DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);    // 关闭 DMA1 通道4, UART1_TX

//		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
//		DMA_SetCurrDataCounter(USARTOne_Rx_DMA_Channel, USART1_RX_MAXBUFF);
//		DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);      // DMA1 通道5, UART1_RX
	}
}

/*******************************************************************************
* 函数功能		     : modbus初始化函数
* 输    入         : 无
* 说    明         :无
*******************************************************************************/ 

void modbus_task_init(void)
{
	UCHAR mySlaveAddress = 0x01;//< 从机地址
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);//参数分别为modbus的工作模式、从机地址、端口号、波特率、奇偶校验设置。
	MyFLASH_ReadByte(FINAL_PAGE_ADDRESS,pdu , MB_RTU_DATA_MAX_SIZE);
	
	if (pdu[car_type] == 0xFFFF)
		{//< 芯片首次初始化
		eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
		Pdu_Init();
		MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		pdu[para_save] = 10;
	}
	else 
		pdu[para_save] = 0;
}

/*******************************************************************************
* 函数功能: USART1传输任务
* 输    入: 无
* 说    明: UART1_TX通过DMA1通道4将usart1_send_data的数据传出，数据为modbus协议格式的帧。
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
		if (usart1_send_flag){//< 回复帧
			DMA_EnableChannel(USARTOne_Tx_DMA_Channel, DISABLE);   // 关闭 DMA1 通道4, UART1_TX
			DMA_SetCurrDataCounter(USARTOne_Tx_DMA_Channel, usart1_send_len);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
			DMA_EnableChannel(USARTOne_Tx_DMA_Channel, ENABLE);    // 开启 DMA1 通道4, UART1_TX
			usart1_send_flag = 0;
			// 发送完成恢复接收
			DMA_EnableChannel(USARTOne_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
			DMA_SetCurrDataCounter(USARTOne_Rx_DMA_Channel, USART1_RX_MAXBUFF);
			DMA_EnableChannel(USARTOne_Rx_DMA_Channel, ENABLE);     // DMA1 通道5, UART1_RX
			Modbus_Respond(&modify);
		}
		if (pdu[para_save] == 1){//< 保存当前参数
			pdu[para_save] = 0;
			MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		}else if (pdu[para_save] == 2){//< 恢复出厂设置
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
函数功能：圆形底盘串口2初始化，代替串口4作为读取电池信息模块
入口参数：baud为通信波特率
返 回 值：无
**************************************************************************/
void Usart2_Init(uint32_t baud)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	/* System Clocks Configuration */
	RCC_EnableAPB1PeriphClk(USARTTwo_CLK, ENABLE);	//使能USART2时钟
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | USARTTwo_GPIO_CLK, ENABLE);//使能GPIO时钟
	GPIO_ConfigPinRemap(GPIO_RMP2_USART2, ENABLE);

	//USART2_TX  
	GPIO_InitStructure.Pin = USARTTwo_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	//USART2_RX	  
	GPIO_InitStructure.Pin = USARTTwo_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	//485 enable	  
	GPIO_InitStructure.Pin = USARTTwo_CTS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTTwo_GPIO, &GPIO_InitStructure);

	/* USARTTwo configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(USARTTwo, &USART_InitStructure);

	//UartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USARTTwo_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	USART2_DMA_Config();     // DMA配置
	USART_ConfigInt(USARTTwo, USART_INT_IDLEF, ENABLE);  // 使能空闲中断

	USART_Enable(USARTTwo, ENABLE);
}

/*******************************************************************************
 *  @brief  串口2 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note     USART2_TX-> DMA1 Channe07; USART2_RX-> DMA1 Channe06
*******************************************************************************/
void USART2_DMA_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(USARTTwo_Tx_DMA_Channel);  // DMA1 通道7, USART2_TX
	DMA_DeInit(USARTTwo_Rx_DMA_Channel);  // DMA1 通道6, USART2_RX
	DMA_StructInit(&DMA_InitStruct);
	RCC_EnableAHBPeriphClk(USARTTwo_DMAx_CLK, ENABLE);

	DMA_InitStruct.PeriphAddr = USARTTwo_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;

	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTTwo_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART2_TX, DMA1, USARTTwo_Tx_DMA_Channel, ENABLE);

	// 配置 DMA1 通道6, USART2_RX
	DMA_InitStruct.PeriphAddr = USARTTwo_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_recv_data;  // 内存地址,用UART4的统一命名
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
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

	// 配置串口2的DMA发送中断
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; 	// 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);

	// 配置 DMA2 通道5, UART4_TX 传输完成中断
	DMA_ConfigInt(USARTTwo_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(USARTTwo, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);     // 禁止接收
	DMA_EnableChannel(USARTTwo_Tx_DMA_Channel, DISABLE);     // 禁止发送
}

/*******************************************************************************
* 函数功能: USART2_TX 传输完成中断
* 输    入: 无
* 说    明: DMA1 通道7, USART2_TX 传输完成中断
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC7, DMA1) != RESET) { // DMA1 通道7, USART2_TX 传输完成
		DMA_ClrIntPendingBit(DMA1_INT_TXC7, DMA1);        // 清除中断
		uart4_send_flag = 1;
		GPIO_ResetBits(USARTTwo_GPIO, USARTTwo_CTS);
		DMA_EnableChannel(USARTTwo_Tx_DMA_Channel, DISABLE);    // 关闭 DMA2 通道5, UART4_TX
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);    // DMA2 通道3, UART4_RX
		DMA_SetCurrDataCounter(USARTTwo_Rx_DMA_Channel, UART4_RX_MAXBUFF);
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, ENABLE);     // DMA2 通道3, UART4_RX
	}
}

/**************************************************************************
* 函数功能：串口2接收中断,接收电池发送过来的数据
* 入口参数：无
* 返 回 值：无
**************************************************************************/
void USARTTwo_IRQHandler(void)
{
	if (USART_GetIntStatus(USARTTwo, USART_INT_IDLEF) != RESET) {
		USARTTwo->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTTwo->DAT; // 清除空闲中断
		//为了与其他类型小车一致，此处用串口4命名的变量
		uart4_recv_flag = 1;                 // 接收标志置1
		uart4_recv_len = UART4_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTTwo_Rx_DMA_Channel);// 统计收到的数据的长度
		DMA_EnableChannel(USARTTwo_Rx_DMA_Channel, DISABLE);    // DMA1 通道6, USART2_RX 
	}
}

/*******************************************************************************USART3*******************************************************************************/
/**************************************************************************
函数功能：串口3初始化，作为上下位机通信
入口参数：无
返 回 值：无
**************************************************************************/
void Usart3_Init(u32 baud)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	RCC_EnableAPB2PeriphClk(USARTThree_GPIO_CLK, ENABLE);	//使能GPIO时钟
	RCC_EnableAPB1PeriphClk(USARTThree_CLK, ENABLE);	//使能USART3时钟
#if (CARMODE == Diff)
	{
		//圆形底盘引脚复用
		RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
		GPIO_ConfigPinRemap(GPIO_PART_RMP_USART3, ENABLE);
	}
#endif
	//USART3_TX  
	GPIO_InitStructure.Pin = USARTThree_TxPin; //PB10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTThree_GPIO, &GPIO_InitStructure);

	//USART3_RX	  
	GPIO_InitStructure.Pin = USARTThree_RxPin;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//浮空输入
	GPIO_InitPeripheral(USARTThree_GPIO, &GPIO_InitStructure);

	/* USART3 configuration*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(USARTThree, &USART_InitStructure);

	//UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USARTThree_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	USART_ConfigInt(USARTThree, USART_INT_IDLEF, ENABLE);  // 使能空闲中断
	Usart3_dma_config();     // DMA配置	

	USART_Enable(USARTThree, ENABLE);
}

/*******************************************************************************
 *  @brief  串口3 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    USART3_TX -> DMA1 Channel2; USART3_RX -> DMA1 Channel3
*******************************************************************************/
void Usart3_dma_config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	RCC_EnableAHBPeriphClk(USARTThree_DMAx_CLK, ENABLE);

	// 配置 DMA1 通道2, USART3_TX
	DMA_InitStruct.PeriphAddr = USARTThree_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart3_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTThree_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART3_TX, DMA1, USARTThree_Tx_DMA_Channel, ENABLE);

	// 配置 DMA1 通道3, USART3_RX
	DMA_InitStruct.PeriphAddr = USARTThree_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)usart3_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
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

	// 配置串口3的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;   // 在 stm32f10x.h 中找 IRQn_Type 枚举
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);

	// 配置 DMA1 通道4, USART3_TX 传输完成中断
	DMA_ConfigInt(USARTThree_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(USARTThree, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(USARTThree_Rx_DMA_Channel, ENABLE);     // 开启接收
	DMA_EnableChannel(USARTThree_Tx_DMA_Channel, DISABLE);    // 禁止发送
}

/**************************************************************************
* 函数功能		   	 : USART3_RX 接收完成中断
* 输    入         : 无
* 说    明         : DMA1 通道3, USART3_RX 接收完成中断
**************************************************************************/
void USARTThree_IRQHandler(void)
{	
	if (USART_GetIntStatus(USARTThree, USART_INT_IDLEF)){
		USART3->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USART3->DAT; // 清除空闲中断
		DMA_EnableChannel(USARTThree_Rx_DMA_Channel, DISABLE);				//暂停接收
		usart3_recv_len = USART3_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTThree_Rx_DMA_Channel);
		usart3_recv_flag = 1;
	}
}

/*******************************************************************************
* 函数功能: USART3_TX 传输完成中断
* 输    入: 无
* 说    明: DMA1 通道2, UART3_TX 传输完成中断
*******************************************************************************/ 
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC2, DMA1)){ //等待DMA传输完成, UART3_TX 传输完成中断为1
		DMA_ClrIntPendingBit(DMA1_INT_TXC2, DMA1);     // 清除中断
		DMA_EnableChannel(USARTThree_Tx_DMA_Channel, DISABLE);        // 关闭 DMA1 通道2, UART3_TX
		usart3_send_flag = 1;
	}
}

/**************************************************************************
函数功能：处理上位机发送过来的数据
入口参数：无
返 回 值：无
**************************************************************************/
void Usart3_Recv(void)
{
	if(ROS_Count < 10000){//防止数据溢出
		ROS_Count++;
	}
	if((pdu[control_mode] != control_mode_remote)){						
		if(usart3_recv_flag){	
			usart3_recv_flag = 0;
			rt_memcpy(Receive_Data, usart3_recv_data, RECEIVE_DATA_SIZE);	
			u8 temp11 = Check_Sum(RECEIVE_DATA_SIZE - 2,0);
			if ((usart3_recv_data[0] == FRAME_HEADER) &&
				(usart3_recv_data[RECEIVE_DATA_SIZE - 1] == FRAME_TAIL) &&
				(Receive_Data[RECEIVE_DATA_SIZE -2] == Check_Sum(RECEIVE_DATA_SIZE - 2, 0))){	//验证数据包的长度、帧头帧尾
				pdu[control_mode] = control_mode_ros;	
				ROS_Count = 0;
				ROS_RecvFlag = true;
				robot_control.ctrl = Receive_Data[1];
			}else{		
				CheckSum_sdo[0][0] = Check_Sum(RECEIVE_DATA_SIZE - 2, 0);
				Add_Sdo_Linked_List(CHECKSUM_ID, CheckSum_sdo, sizeof(CheckSum_sdo)/sizeof(CheckSum_sdo[0]));//通过CAN显示争取的Sum
				if(ROS_Count > 300)	pdu[control_mode] = control_mode_unknown;//1500ms未接收到正确ros消息，则置零控制模式
			}
			DMA_SetCurrDataCounter(USARTThree_Rx_DMA_Channel, USART3_RX_MAXBUFF);//传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
			DMA_EnableChannel(USARTThree_Rx_DMA_Channel, ENABLE);     		// DMA1 通道2, UART3_RX
		}else if(ROS_Count > 300){	//1500ms未接收到ros消息，则置零控制模式
			pdu[control_mode] = control_mode_unknown;
		}
	}
}

/**************************************************************************
函数功能：串口3(ROS)发送数据
入口参数：无
返回  值：无
**************************************************************************/
void Usart3_Send(void)
{
	if (usart3_send_flag){
		usart3_send_flag = 0;	
		
		Data_Assignment(); 
		
		rt_memcpy(usart3_send_data, Send_Data.buffer, USART3_TX_MAXBUFF);
		DMA_SetCurrDataCounter(USARTThree_Tx_DMA_Channel, USART3_TX_MAXBUFF); 
		DMA_EnableChannel(USARTThree_Tx_DMA_Channel, ENABLE);    // 开启DMA1 通道2，USART3_TX	
	}
}

/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void Data_Assignment(void)
{
	Send_Data.d.Frame_Header = FRAME_HEADER;			//1个字节 = FRAME_HEADER; //帧头(固定值)      		
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
	Send_Data.d.Odometry2 = pdu[Odom2ForRobot];       	//2个字节	
	Send_Data.d.Check_SUM = Check_Sum(USART3_TX_MAXBUFF-2, 1);;
	Send_Data.d.Frame_Tail = FRAME_TAIL; //帧尾（固定值）
}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：23位为校验位，结果是数组1-22的异或结果；后一个参数为发送或是接收校验位
返 回 值：检验位
**************************************************************************/
unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode) 
{
	unsigned char check_sum = 0, k;
	// 数据缓冲区的选择
	unsigned char* dataBuffer = (Mode == 1) ? Send_Data.buffer : Receive_Data;
	for (k = 0; k < Count_Number; k++) {check_sum = check_sum ^ dataBuffer[k];}
	return check_sum;
}

/**************************************************************************
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
short ConvertBytesToShort(u8 highByte,u8 lowByte)
{
    return (((short)highByte << 8) + (short)lowByte);    
}

/*******************************************************************************UART4*******************************************************************************/
/**************************************************************************
函数功能：串口4初始化，作为读取电池信息模块
入口参数：baud为通信波特率
返 回 值：无
**************************************************************************/
#if(CARMODE != Diff)
void Uart4_Init(uint32_t baud)         
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;

	/* System Clocks Configuration */
	RCC_EnableAPB1PeriphClk(UARTFour_CLK, ENABLE);	//使能UART4时钟
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | UARTFour_GPIO_CLK, ENABLE);//使能GPIO时钟
	
	/* Configure the GPIO ports */
	GPIO_ConfigPinRemap(AFIO_RMP_CFG_SW_JTAG_CFG_DISABLE | GPIO_RMP_SW_JTAG_DISABLE, ENABLE);

	//UART4_TX  
	GPIO_InitStructure.Pin = UARTFour_TxPin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitPeripheral(UARTFour_GPIO, &GPIO_InitStructure);

	//UART4_RX	  
	GPIO_InitStructure.Pin = UARTFour_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitPeripheral(UARTFour_GPIO, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_RMP2_UART4, ENABLE);

	//485 enable	  
	GPIO_InitStructure.Pin = UARTFour_485enPin; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
	GPIO_InitPeripheral(UARTFour_485en_GPIO, &GPIO_InitStructure);

	/* UARTFour configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(UARTFour, &USART_InitStructure);

	//UartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	Uart4_Dma_Config();     // DMA配置
	USART_ConfigInt(UARTFour, USART_INT_IDLEF, ENABLE);  // 使能空闲中断

	USART_Enable(UARTFour, ENABLE);
}

/*******************************************************************************
 *  @brief  串口4 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART4_TX -> DMA2_CH5; UART3_RX -> DMA2_CH3
*******************************************************************************/
void Uart4_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(UARTFour_Tx_DMA_Channel);  // DMA2 通道5, UART4_TX
	DMA_DeInit(UARTFour_Rx_DMA_Channel);  // DMA2 通道3, UART4_RX
	DMA_StructInit(&DMA_InitStruct);
	RCC_EnableAHBPeriphClk(UARTFour_DMAx_CLK, ENABLE);

	// 配置 DMA2 通道5, UART4_TX
	DMA_InitStruct.PeriphAddr = UARTFour_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
 
	DMA_Init(UARTFour_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA2_REMAP_UART4_TX, DMA2, UARTFour_Tx_DMA_Channel, ENABLE);

	// 配置 DMA2 通道3, UART4_RX
	DMA_InitStruct.PeriphAddr = UARTFour_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
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

	// 配置串口4的DMA发送中断
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Channel5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; 	// 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);

	// 配置 DMA2 通道5, UART4_TX 传输完成中断
	DMA_ConfigInt(UARTFour_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	USART_EnableDMA(UARTFour, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

	DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);     // 禁止接收
	DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);     // 禁止发送
}

/*******************************************************************************
* 函数功能: UART4_TX 传输完成中断
* 输    入: 无
* 说    明: DMA1 通道2, UART4_TX 传输完成中断
*******************************************************************************/ 
void DMA2_Channel5_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA2_INT_TXC5, DMA2) != RESET){ // DMA2 通道5, UART4_TX 传输完成
		DMA_ClrIntPendingBit(DMA2_INT_TXC5, DMA2);     // 清除中断
		uart4_send_flag = 1;
		GPIO_ResetBits(UARTFour_485en_GPIO, UARTFour_485enPin);
		DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);    // 关闭 DMA2 通道5, UART4_TX
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);    // DMA2 通道3, UART4_RX
		DMA_SetCurrDataCounter(UARTFour_Rx_DMA_Channel, UART4_RX_MAXBUFF);
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, ENABLE);     // DMA2 通道3, UART4_RX
	}
}

/**************************************************************************
* 函数功能：串口4接收中断,接收电池发送过来的数据
* 入口参数：无
* 返 回 值：无
**************************************************************************/
void UARTFour_IRQHandler(void)
{
	if (USART_GetIntStatus(UARTFour, USART_INT_IDLEF) != RESET){
		UARTFour->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		UARTFour->DAT; // 清除空闲中断
		uart4_recv_flag = 1;                 // 接收标志置1
		uart4_recv_len = UART4_RX_MAXBUFF - DMA_GetCurrDataCounter(UARTFour_Rx_DMA_Channel);// 统计收到的数据的长度
		DMA_EnableChannel(UARTFour_Rx_DMA_Channel, DISABLE);    // DMA2 通道3, UART4_RX
	}
}
#endif
/**************************************************************************
* 函数功能：JTAG模式设置,用于设置JTAG的模式
* 入口参数：mode: jtag,swd模式设置;00, 全使能;01, 使能SWD;10, 全关闭;
* 说   明 ：JTAG,SWD但没有NJTRST选项没有写入  
**************************************************************************/	  
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2PCLKEN |= 1 << 0;     //寮�鍚?杈呭姪鏃堕挓	   
	AFIO->RMP_CFG &= 0XF8FFFFFF; //娓呴櫎MAPR鐨刐26:24]
	AFIO->RMP_CFG |= temp;       //璁剧疆jtag妯″紡
}

/*******************************************************************************UART5*******************************************************************************/
/**************************************************
* 函数功能：	串口5初始化函数，作为航模信号接收
* 参    数：  unBound代表通信波特率，需为100k
* 返 回 值：  无
* 100k波特率，8位数据位(stm32-选择9位)，2位停止位，偶校验（EVEN)，无控流，25个字节。
**************************************************/
void Uart5_Init(unsigned int unBound)
{
	//GPIO端口设置
  	GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStructure;	

	RCC_EnableAPB1PeriphClk(UARTFive_CLK, ENABLE);	//使能UART5，GPIOA时钟
  	RCC_EnableAPB2PeriphClk(UARTFive_GPIO_CLK, ENABLE);

  	//UART5_RX	  GPIOB.14初始化
  	GPIO_InitStructure.Pin = UARTFive_RxPin;//PB14
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  	GPIO_InitPeripheral(UARTFive_GPIO, &GPIO_InitStructure);//初始化GPIOB.14  
#if (CARMODE != Diff)
	{
		GPIO_ConfigPinRemap(GPIO_RMP1_UART5, ENABLE);
	}
#endif

   	//UART 初始化设置
	USART_InitStructure.BaudRate = unBound;//串口波特率
	USART_InitStructure.WordLength = USART_WL_9B;//字长为8位数据格式
	USART_InitStructure.StopBits = USART_STPB_2;//2个停止位
	USART_InitStructure.Parity = USART_PE_NO;//偶校验位
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;//无硬件数据流控制
	USART_InitStructure.Mode = USART_MODE_RX;	//收模式
  	USART_Init(UARTFive, &USART_InitStructure); //初始化串口5

  	//UART5 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UARTFive_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器  
	Uart5_Dma_Config();     // DMA配置
	USART_ConfigInt(UARTFive, USART_INT_IDLEF, ENABLE);//开启串口空闲中断
//	//test
//	USART_ConfigInt(UARTFive, USART_INT_OREF, ENABLE);//开启串口溢出中断
//	USART_ConfigInt(UARTFive, USART_INT_ERRF, ENABLE);//开启串口错误中断
//	USART_ConfigInt(UARTFive, USART_INT_NEF, ENABLE);//开启串口错误中断
//	USART_ConfigInt(UARTFive, USART_INT_FEF, ENABLE);//开启串口错误中断

	USART_Enable(UARTFive, ENABLE);                    //使能串口5 
}

/**************************************************
* 函数功能：	串口5初始化函数，作为航模信号接收
* 参    数：  unBound代表通信波特率，需为100k
* 返 回 值：  无
**************************************************/
void Uart5_Dma_Config(void)
{
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(UARTFive_Rx_DMA_Channel);  // DMA1 通道8, UART5_RX
	DMA_StructInit(&DMA_InitStruct);
	// 配置 DMA1 通道8, UART5_RX
	DMA_InitStruct.PeriphAddr = UARTFive_DR_Base;   	// 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)Uart5_Buffer;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;    // 外设到内存
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

	DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // 开启接收
}

/**************************************************
* 函数功能：	信号接收中断处理函数
* 入口参数：  无
* 返 回 值：  无
**************************************************/
void UARTFive_IRQHandler()
{
	if (USART_GetIntStatus(UARTFive, USART_INT_IDLEF)) //空闲中断产生 
	{
		UARTFive->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		UARTFive->DAT; // 清除空闲中断
		Sbus_Data_Parsing_Flag = 1;                // 接收标志置1
		pdu[control_mode] = control_mode_remote;                // 航模控制
		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, DISABLE);    // DMA1 通道3, UART3_RX
		DMA_SetCurrDataCounter(UARTFive_Rx_DMA_Channel, UART5_RX_MAXBUFF);
		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // DMA1 通道3, UART3_RX
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
//	else if (USART_GetIntStatus(UARTFive, USART_INT_IDLEF) != RESET) //中断产生 
//	{
//		UARTFive->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
//		UARTFive->DAT; // 清除空闲中断
//		Sbus_Data_Parsing_Flag = 1;                // 接收标志置1
//		pdu[control_mode] = control_mode_remote;   // 航模控制
//		// 统计收到的数据的长度
//		//ucRcvCount = Max_BUFF_Len - DMA_GetCurrDataCounter(UARTFive_Rx_DMA_Channel);
//		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, DISABLE);    // DMA1 通道3, UART3_RX
//		DMA_SetCurrDataCounter(UARTFive_Rx_DMA_Channel, Max_BUFF_Len);
//		DMA_EnableChannel(UARTFive_Rx_DMA_Channel, ENABLE);     // DMA1 通道3, UART3_RX
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_NEF) != RESET)
//	{
//		UARTFive->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
//		UARTFive->DAT; // 清除空闲中断
//	}
//	else if (USART_GetIntStatus(UARTFive, USART_INT_FEF) != RESET)
//	{
//		UARTFive->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
//		UARTFive->DAT; // 清除空闲中断
//	}
}

/**************************************************
* 函数功能：	对整型数据取绝对值
**************************************************/
uint32_t Abs_int(int nValue)
{
	if(nValue < 0)	return (-nValue);
	else						return nValue;
}

/**************************************************************************
函数功能：485地址表初始化
入口参数：无
返回  值：无
**************************************************************************/
void Pdu_Init(void)
{
	enum enum_CarType CarType = CARMODE;	//小车类型
	enum enum_motor_type motor_type[MAX_MOTOR_NUMBER] = {0};	//电机类型
	enum enum_sport_mode sport_mode[MAX_MOTOR_NUMBER] = {0};	//电机运动模式
	uint16_t CAN_id[MAX_MOTOR_NUMBER] = {0};					//电机canid
	uint16_t reduction_ratio[MAX_MOTOR_NUMBER] = {0};			//电机减速比
	uint16_t direction[MAX_MOTOR_NUMBER] = {0};					//电机方向
	uint16_t acceleration_time[MAX_MOTOR_NUMBER] = {0};			//加速时间（ZLAC）
	uint16_t deceleration_time[MAX_MOTOR_NUMBER] = {0};			//减速时间（ZLAC）
	uint16_t profile_rpm[MAX_MOTOR_NUMBER] = {0};				//最大速度（WANZE）
	uint16_t profile_acce[MAX_MOTOR_NUMBER] = {0};				//加速度（WANZE）
	uint16_t profile_jerk[MAX_MOTOR_NUMBER] = {0};				//加加速度（WANZE）

	struct_RobotBasePara AkmCarBasePara = {
		.car_type = Akm_Car,					
		.car_product_number = AKM_PRODUCT_NUM,	
		.car_version = AKM_VERSION,				
		.car_length = 11460,					
		.car_width = 7220,						
		.car_height = 3940,						
		.car_wheelbase = 6500,					//轴距
		.car_tread = 5800,					    //轮距
		.car_ground_clearance = 890,			//底盘高度
		.wheel_radius = 1650,					//车轮半径
		.gross_max = 80,						//净重量
		.rated_load = 70,						//额定载荷
		.motor_number = 3,						//电机数量
		.driving_method = rear_drive,			//驱动方式
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
		.motor_number = 1,//实际为2，这里是统一方便，写1是为了好实现pdo，驱动器一拖二
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
	//pdu部分数据初始化
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
	pdu[i++] = 5000;//机器人加速度系数
	pdu[i++] = car_direct_forward;//机器人前进方向
	pdu[i++] = car_direct_forward;//机器人转弯方向
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

// 写入所有可读可写参数
void Modbus_Respond(MBModify* modify) 
{
	if (modify->is_modify == 0)return;
	if (pdu[para_save] == 3)
	{//< 当前修改参数写入flash
		pdu[para_save] = 0;
		switch (modify->modify_addr)
		{
		case motor1_direction:
			pdu[motor1_direction];
			break;
		}
	}

}


