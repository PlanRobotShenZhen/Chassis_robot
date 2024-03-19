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

/*--------上下位机通信格式数据usart3-----------*/
SEND_DATA Send_Data;//发送数据的结构体
RECEIVE_DATA Receive_Data;//接收数据的结构体
uint8_t uart1_recv_data[USART1_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t uart1_send_data[USART1_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint32_t uart1_recv_flag = 0;        // 接收完成标志位
uint32_t uart1_recv_len = 0;         // 接收的数据长度
uint32_t uart1_send_len = 0;         // 发送的数据长度
uint32_t uart1_send_flag = 0;        // 发送完成标志位
#define USART3_RX_MAXBUFF 25
#define USART3_TX_MAXBUFF 50
uint8_t uart3_recv_data[USART3_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t uart3_send_data[USART3_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint8_t uart3_recv_flag = 0;        // 接收完成标志位
uint8_t uart3_recv_len = 0;         // 接收的数据长度
uint8_t uart3_send_len = 0;         // 发送的数据长度
uint8_t uart3_send_flag = 1;        // 发送完成标志位


uint8_t uart4_recv_data[USART4_RX_MAXBUFF] = { 0 };  // 接收数据缓冲区
uint8_t uart4_send_data[USART4_TX_MAXBUFF] = { 0 };  // 发送数据缓冲区
uint32_t uart4_recv_flag = 0;        // 接收完成标志位
uint32_t uart4_recv_len = 0;         // 接收的数据长度
uint32_t uart4_send_len = 0;         // 发送的数据长度
uint32_t uart4_send_flag = 1;        // 发送完成标志位
/*--------航模数据解析相关全局变量的定义-----------*/
SBUS_CH_Struct tagSBUS_CH;
unsigned char Uart5_Buffer[Max_BUFF_Len] = {0};  
unsigned int  ucRcvCount = 0; 
uint8_t ucRcvReady = 0;
uint8_t is_connect = 0;
int g_nVelocity = 0;       //前后加速速度                   
int g_nDirector = 0;       //转向

int g_nLastVelocity = 0;          //存放当前的速度和上一刻的速度
int g_nCurrentVelocity = 0;

unsigned char g_ucLightOnFlag = 0; //车灯开启标志
float g_fltRecv_Vel_X = 0.0;                        // 串口接收到的速度数据
float g_fltRecv_Vel_Y = 0.0;
float g_fltRecv_Vel_Z = 0.0;
static uint16_t* pdu;
int remote_off_line_check = 0;

void Modbus_Respond(MBModify* modify);
/**************************************************************************
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low)
{
	short transition; //数据转换的中间变量
	transition = ((High << 8) + Low); //将高8位和低8位整合成一个16位的short型数据
	return transition * 0.001f;    //发送端将数据发送前做了一个*1000的单位换算，这里接收数据后需要还原单位
}
/**************************************************************************
函数功能：串口收发收据任务函数,串口3发送数据到上位机，中断接收数据
入口参数：无
返回  值：无
**************************************************************************/
void DATA_task(void *pvParameters)
{
	
   while(1)
    {	
		rt_thread_delay(50);   //< 5ms
			if (uart3_recv_flag)
			{
				uart3_recv_flag = 0;
				if (uart3_recv_len == 11)
				{
					if (Receive_Data.buffer[10] == FRAME_TAIL) //验证数据包的尾部校验信息
					{
						if (Receive_Data.buffer[9] == Check_Sum(&Receive_Data.buffer[0],9, 0))	 //数据校验位计算，模式0是发送数据校验
						{
							g_ucRos_Flag = 1;                   // 航模开启的时候，给这个变量赋值为0
							if (g_eControl_Mode!= CONTROL_MODE_REMOTE)
							{
								remote_off_line_check = 0;
								g_eControl_Mode = CONTROL_MODE_ROS;   // 为ROS上位机控制
								robot_control.ctrl = Receive_Data.buffer[1];
								pdu[light_control] = Receive_Data.buffer[2];
							}
							g_fltRecv_Vel_X =((int16_t)(Receive_Data.buffer[3]<<8| Receive_Data.buffer[4])) *0.001f;
							g_fltRecv_Vel_Y =((int16_t)(Receive_Data.buffer[5]<<8| Receive_Data.buffer[6])) *0.001f;
							g_fltRecv_Vel_Z =((int16_t)(Receive_Data.buffer[7]<<8| Receive_Data.buffer[8])) *0.001f;
						}
					}
				}
			}

			if (uart3_send_flag == 1)
			{//< 回复帧
				uart3_send_flag = 0;
				Data_transition(); //对要进行发送的数据进行赋值
			}
		}
}

/**
 *  @brief  串口5 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART5_RX -> DMA1 Channel8; 
 */
void uart5_dma_config(void)
{
	DMA_InitType DMA_InitStruct;
	DMA_DeInit(USARTe_Rx_DMA_Channel);  // DMA1 通道8, UART5_RX
	DMA_StructInit(&DMA_InitStruct);
	// 配置 DMA1 通道8, UART5_RX
	DMA_InitStruct.PeriphAddr = USARTe_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)Uart5_Buffer;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
	DMA_InitStruct.BufSize = Max_BUFF_Len;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTe_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_UART5_RX, DMA1, USARTe_Rx_DMA_Channel, ENABLE);
	/* Enable UART5 DMA Rx request */
	USART_EnableDMA(USARTe, USART_DMAREQ_RX, ENABLE);
	DMA_EnableChannel(USARTe_Rx_DMA_Channel, ENABLE);     // 开启接收
}
/**************************************************
* 函数功能：	串口5初始化函数，作为航模信号接收
* 参    数：  unBound代表通信波特率，需为100k
* 返 回 值：  无
* 100k波特率，8位数据位(stm32-选择9位)，2位停止位，偶校验（EVEN)，无控流，25个字节。
**************************************************/
void Usart5_Init(unsigned int unBound)
{
	//GPIO端口设置
    GPIO_InitType GPIO_InitStructure;
	USART_InitType USART_InitStructure;
	NVIC_InitType NVIC_InitStructure;	
	RCC_EnableAPB1PeriphClk(USARTe_CLK, ENABLE);	//使能USART5，GPIOA时钟
    RCC_EnableAPB2PeriphClk(USARTe_GPIO_CLK, ENABLE);
  //USART5_RX	  GPIOB.14初始化
    GPIO_InitStructure.Pin = USARTe_RxPin;//PB14
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_InitPeripheral(USARTe_GPIO, &GPIO_InitStructure);//初始化GPIOB.14  
	GPIO_ConfigPinRemap(GPIO_RMP1_UART5, ENABLE);
  //USART5 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USARTe_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器  
   //USART 初始化设置
	USART_InitStructure.BaudRate = unBound;//串口波特率
	USART_InitStructure.WordLength = USART_WL_9B;//字长为8位数据格式
	USART_InitStructure.StopBits = USART_STPB_2;//一个停止位
	USART_InitStructure.Parity = USART_PE_NO;//偶校验位
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;//无硬件数据流控制
	USART_InitStructure.Mode = USART_MODE_RX;	//收模式
    USART_Init(USARTe, &USART_InitStructure); //初始化串口5
	uart5_dma_config();     // DMA配置
	//USART_ConfigInt(UART5, USART_INT_RXDNE, DISABLE);//关闭串口接收中断
	USART_ConfigInt(USARTe, USART_INT_IDLEF, ENABLE);//开启串口空闲中断
	USART_ConfigInt(USARTe, USART_INT_OREF, ENABLE);//开启串口溢出中断
	USART_ConfigInt(USARTe, USART_INT_ERRF, ENABLE);//开启串口错误中断
	USART_ConfigInt(USARTe, USART_INT_NEF, ENABLE);//开启串口错误中断
	USART_ConfigInt(USARTe, USART_INT_FEF, ENABLE);//开启串口错误中断
	USART_Enable(USARTe, ENABLE);                    //使能串口5 
	tagSBUS_CH.CH1 = 1023;
	tagSBUS_CH.CH2 = 1023;
	tagSBUS_CH.CH3 = 1023;
	tagSBUS_CH.CH4 = 1023;
}

/**
 *  @brief  DMA1 通道2, UART3_TX 中断控制器配置
 *  @param  无
 *  @retval 无
 *  @note   中断优先级分组全工程只配置一次，在 main 函数最开始进行配置
 *  @note   中断处理函数在 CMSIS/stm32f10x_it.c 中进行处理
 */
void DMA1_Channel2_nvic_config(void)
{
	NVIC_InitType NVIC_InitStruct;

	// 配置串口1的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel2_IRQn;   // 在 stm32f10x.h 中找 IRQn_Type 枚举
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);
}
/**
 *  @brief  串口3 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART3_TX -> DMA1 Channel2; UART3_RX -> DMA1 Channel3
 */
void uart3_dma_config(void)
{
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(USARTz_Tx_DMA_Channel);  // DMA1 通道2, UART3_TX
	DMA_DeInit(USARTz_Rx_DMA_Channel);  // DMA1 通道3, UART3_RX

	DMA_StructInit(&DMA_InitStruct);
	// 配置 DMA1 通道4, UART1_TX
	DMA_InitStruct.PeriphAddr = USARTz_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart3_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTz_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART3_TX, DMA1, USARTz_Tx_DMA_Channel, ENABLE);

	// 配置 DMA1 通道5, UART1_RX
	DMA_InitStruct.PeriphAddr = USARTz_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart3_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
	DMA_InitStruct.BufSize = USART3_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTz_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART3_RX, DMA1, USARTz_Rx_DMA_Channel, ENABLE);



	DMA1_Channel2_nvic_config();
	// 配置 DMA1 通道4, UART1_TX 传输完成中断
	DMA_ConfigInt(USARTz_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	/* Get Current Data Counter value before transfer begins */

	/* Enable USARTz DMA Rx and TX request */
	USART_EnableDMA(USARTz, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
	DMA_EnableChannel(USARTz_Rx_DMA_Channel, ENABLE);     // 开启接收
	DMA_EnableChannel(USARTz_Tx_DMA_Channel, DISABLE);    // 禁止发送
}

/**************************************************************************
函数功能：串口3初始化，作为上下位机通信
入口参数：baud为通信波特率
是在PCB板中的第二个USB接口
返 回 值：无
**************************************************************************/
void Usart3_Init(uint32_t baud)
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;
	/* System Clocks Configuration */
	RCC_EnableAPB2PeriphClk(USARTz_GPIO_CLK, ENABLE);	//使能GPIO时钟
	RCC_EnableAPB1PeriphClk(USARTz_CLK, ENABLE);	//使能USART3时钟

	/* Configure the GPIO ports */
	//USART_TX  
	GPIO_InitStructure.Pin = USARTz_TxPin; //PB10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);
	//USART_RX	  
	GPIO_InitStructure.Pin = USARTz_RxPin;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);

	/* USARTz configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

	/* Configure USARTz */
	USART_Init(USARTz, &USART_InitStructure);

	//UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	uart3_dma_config();     // DMA配置
	USART_ConfigInt(USARTz, USART_INT_IDLEF, ENABLE);  // 使能空闲中断

	/* Enable the USARTz */
	USART_Enable(USARTz, ENABLE);
}
/**
 *  @brief  串口4 DMA初始化配置
 *  @param  无
 *  @retval 无
 *  @note    UART4_TX -> DMA2 Channel5; UART4_RX -> DMA2 Channel3
 */
void uart4_dma_config(void)
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStruct;

	DMA_DeInit(USARTb_Tx_DMA_Channel);  // DMA2 通道5, UART4_TX
	DMA_DeInit(USARTb_Rx_DMA_Channel);  // DMA2 通道3, UART4_RX

	DMA_StructInit(&DMA_InitStruct);
	// 配置 DMA2 通道5, UART4_TX
	DMA_InitStruct.PeriphAddr = USARTb_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
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
	DMA_Init(USARTb_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA2_REMAP_UART4_TX, DMA2, USARTb_Tx_DMA_Channel, ENABLE);

	// 配置 DMA2 通道3, UART4_RX
	DMA_InitStruct.PeriphAddr = USARTb_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart4_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
	DMA_InitStruct.BufSize = USART4_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTb_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA2_REMAP_UART4_RX, DMA2, USARTb_Rx_DMA_Channel, ENABLE);

	// 配置串口4的DMA发送中断
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Channel5_IRQn;   //
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_Init(&NVIC_InitStruct);
	// 配置 DMA2 通道5, UART4_TX 传输完成中断
	DMA_ConfigInt(USARTb_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	/* Get Current Data Counter value before transfer begins */

	/* Enable USARTz DMA Rx and TX request */
	USART_EnableDMA(USARTb, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
	DMA_EnableChannel(USARTb_Rx_DMA_Channel, DISABLE);     // 禁止接收
	DMA_EnableChannel(USARTb_Tx_DMA_Channel, DISABLE);    // 禁止发送
}
//JTAG模式设置,用于设置JTAG的模式
//mode: jtag,swd模式设置;00, 全使能;01, 使能SWD;10, 全关闭;
//JTAG,SWD但没有NJTRST选项没有写入		  
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2PCLKEN |= 1 << 0;     //开启辅助时钟	   
	AFIO->RMP_CFG &= 0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->RMP_CFG |= temp;       //设置jtag模式
}

/**************************************************************************
函数功能：串口4初始化，作为读取电池信息模块
入口参数：baud为通信波特率
返 回 值：无
**************************************************************************/
void Usart4_Init(uint32_t baud)         //usart4
{
	GPIO_InitType GPIO_InitStructure;
	NVIC_InitType NVIC_InitStructure;
	USART_InitType USART_InitStructure;
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);
	GPIO_ConfigPinRemap(AFIO_RMP_CFG_SW_JTAG_CFG_DISABLE, ENABLE);
	GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_DISABLE, ENABLE);

	/* System Clocks Configuration */
	RCC_EnableAPB2PeriphClk(USARTb_GPIO_CLK, ENABLE);	//使能GPIO时钟
	RCC_EnableAPB1PeriphClk(USARTb_CLK, ENABLE);	//使能USART4时钟
	RCC_EnableAHBPeriphClk(USARTb_DMAx_CLK, ENABLE);
	/* Configure the GPIO ports */
	//USART_TX  
	GPIO_InitStructure.Pin = USARTb_TxPin; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTb_GPIO, &GPIO_InitStructure);
	//USART_RX	  
	GPIO_InitStructure.Pin = USARTb_RxPin;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_InitPeripheral(USARTb_GPIO, &GPIO_InitStructure);
	GPIO_ConfigPinRemap(GPIO_RMP2_UART4, ENABLE);
	/*****************************************/
	//485 enable	  
	GPIO_InitStructure.Pin = USARTb_485enPin; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
	GPIO_InitPeripheral(USARTb_485en_GPIO, &GPIO_InitStructure);
	/* USARTb configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

	/* Configure USARTb */
	USART_Init(USARTb, &USART_InitStructure);

	//UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USARTb_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	uart4_dma_config();     // DMA配置
	USART_ConfigInt(USARTb, USART_INT_IDLEF, ENABLE);  // 使能空闲中断

	/* Enable the USARTb */
	USART_Enable(USARTb, ENABLE);
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
* 函数功能：	将sbus信号转换为通道值
* 参    数：  ucBuf为接收到的串口数据
* 返 回 值：  0代表成功，1代表失败
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
* 函数功能：	信号接收中断处理函数
* 入口参数：  无
* 返 回 值：  无
**************************************************/
void USARTe_IRQHandler() 
{
	if (USART_GetIntStatus(USARTe, USART_INT_ERRF) != RESET)
	{// USART_FLAG_ERRF
		USART_ClrFlag(USARTe, USART_INT_ERRF);
		USART_ReceiveData(USARTe);
	}
	else if (USART_GetIntStatus(USARTe, USART_INT_OREF) != RESET)
	{// USART_FLAG_ORE
		USART_ClrFlag(USARTe, USART_INT_OREF);
		USART_ReceiveData(USARTe);
	}
	else if (USART_GetIntStatus(USARTe, USART_INT_IDLEF) != RESET) //中断产生 
	{
		USARTe->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTe->DAT; // 清除空闲中断
		ucRcvReady = 1;                // 接收标志置1
		g_eControl_Mode = CONTROL_MODE_REMOTE; //航模遥控方式选择
		// 统计收到的数据的长度
		ucRcvCount = Max_BUFF_Len - DMA_GetCurrDataCounter(USARTe_Rx_DMA_Channel);
		DMA_EnableChannel(USARTe_Rx_DMA_Channel, DISABLE);    // DMA1 通道3, UART3_RX
		DMA_SetCurrDataCounter(USARTe_Rx_DMA_Channel, Max_BUFF_Len);
		DMA_EnableChannel(USARTe_Rx_DMA_Channel, ENABLE);     // DMA1 通道3, UART3_RX
	}
	else if (USART_GetIntStatus(USARTe, USART_INT_NEF) != RESET)
	{
		USARTe->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTe->DAT; // 清除空闲中断
	}
	else if (USART_GetIntStatus(USARTe, USART_INT_FEF) != RESET)
	{
		USARTe->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTe->DAT; // 清除空闲中断
	}

}


// DMA1 通道2, UART3_TX 传输完成中断
void DMA1_Channel2_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC2, DMA1) != RESET) // DMA1 通道2, UART3_TX 传输完成
	{
		DMA_ClrIntPendingBit(DMA1_INT_TXC2, DMA1);     // 清除中断

		uart3_send_flag = 1;
		DMA_EnableChannel(DMA1_CH2, DISABLE);        // 关闭 DMA1 通道2, UART3_TX
	}
}
/**************************************************************************
函数功能：串口3接收中断,接收上位机发送过来的数据
入口参数：无
返 回 值：无
**************************************************************************/
void USARTz_IRQHandler(void)
{	
	if (USART_GetIntStatus(USARTz, USART_INT_IDLEF) != RESET)
	{
		USARTz->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTz->DAT; // 清除空闲中断
		uart3_recv_flag = 1;                // 接收标志置1
		// 统计收到的数据的长度
		uart3_recv_len = USART3_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTz_Rx_DMA_Channel);
		memcpy(Receive_Data.buffer, uart3_recv_data, uart3_recv_len);
		DMA_EnableChannel(USARTz_Rx_DMA_Channel, DISABLE);    // DMA1 通道3, UART3_RX
		DMA_SetCurrDataCounter(USARTz_Rx_DMA_Channel, USART3_RX_MAXBUFF);
		DMA_EnableChannel(USARTz_Rx_DMA_Channel, ENABLE);     // DMA1 通道3, UART3_RX
	}
}


// DMA2 通道5, UART4_TX 传输完成中断
void DMA2_Channel5_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA2_INT_TXC5, DMA2) != RESET) // DMA2 通道5, UART4_TX 传输完成
	{
		DMA_ClrIntPendingBit(DMA2_INT_TXC5, DMA2);     // 清除中断
		uart4_send_flag = 1;
		GPIO_ResetBits(USARTb_485en_GPIO, USARTb_485enPin);
		//GPIO_SetBits(USARTb_485en_GPIO, USARTb_485enPin);
		DMA_EnableChannel(USARTb_Tx_DMA_Channel, DISABLE);        // 关闭 DMA2 通道5, UART4_TX
		DMA_EnableChannel(USARTb_Rx_DMA_Channel, DISABLE);    // DMA2 通道3, UART4_RX
		DMA_SetCurrDataCounter(USARTb_Rx_DMA_Channel, USART4_RX_MAXBUFF);
		DMA_EnableChannel(USARTb_Rx_DMA_Channel, ENABLE);     // DMA2 通道3, UART4_RX
	}
}
/**************************************************************************
函数功能：串口4接收中断,接收电池发送过来的数据
入口参数：无
返 回 值：无
**************************************************************************/
void USARTb_IRQHandler(void)
{
	if (USART_GetIntStatus(USARTb, USART_INT_IDLEF) != RESET)
	{
		USARTb->STS; // 清除空闲中断, 由软件序列清除该位(先读USART_SR，然后读USART_DR)
		USARTb->DAT; // 清除空闲中断
		uart4_recv_flag = 1;                // 接收标志置1
		// 统计收到的数据的长度
		uart4_recv_len = USART4_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTb_Rx_DMA_Channel);
		DMA_EnableChannel(USARTb_Rx_DMA_Channel, DISABLE);    // DMA2 通道3, UART4_RX
	}
}
/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void Data_transition(void)
{
	uint8_t i = 0;
	uint8_t si = 0;
	Send_Data.Frame_Header = FRAME_HEADER; //帧头
	Send_Data.Frame_Tail = FRAME_TAIL; //帧尾
	
	switch(g_emCarMode)
	{	
		case Mec_Car:      //麦克纳姆轮小车 
			Send_Data.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/4)*1000; //小车x轴速度
	    Send_Data.Y_speed = ((MOTOR_A.fltFeedBack_Velocity - MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity - MOTOR_D.fltFeedBack_Velocity)/4)*1000; //小车y轴速度
	    Send_Data.Z_speed = ((-MOTOR_A.fltFeedBack_Velocity - MOTOR_B.fltFeedBack_Velocity
																			+MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/4/(Axle_spacing+Wheel_spacing))*1000;//小车z轴速度            
		  break; 
		
    case Omni_Car: //全向轮小车     
			Send_Data.X_speed = ((MOTOR_C.fltFeedBack_Velocity-MOTOR_B.fltFeedBack_Velocity)/2/X_PARAMETER)*1000; //小车x轴速度
	    Send_Data.Y_speed = ((MOTOR_A.fltFeedBack_Velocity*2-MOTOR_B.fltFeedBack_Velocity-MOTOR_C.fltFeedBack_Velocity)/3)*1000; //小车y轴速度
	    Send_Data.Z_speed = ((MOTOR_A.fltFeedBack_Velocity+MOTOR_B.fltFeedBack_Velocity+MOTOR_C.fltFeedBack_Velocity)/3/Omni_turn_radiaus)*1000;//小车z轴速度       
		  break; 
    
		case Akm_Car:   //阿克曼小车
			Send_Data.X_speed = ((MOTOR_A.fltFeedBack_Velocity+MOTOR_B.fltFeedBack_Velocity)/2)*1000; //小车x轴速度
			Send_Data.Y_speed = 0;
			Send_Data.Z_speed = ((MOTOR_B.fltFeedBack_Velocity-MOTOR_A.fltFeedBack_Velocity)/Wheel_spacing)*1000;//小车z轴速度
		  break; 
		
		case Diff_Car:  //两轮差速小车
			Send_Data.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity)/2)*1000; //小车x轴速度
			Send_Data.Y_speed = 0;
			Send_Data.Z_speed = ((MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity)/Wheel_spacing)*1000;//小车z轴速度
			break; 
		
		case FourWheel_Car: //四驱车 
      Send_Data.X_speed = pdu[car_feedback_lin_speed]; //小车x轴速度	  
	    Send_Data.Y_speed = 0;
		  /*Z轴方向上发送的是一个角速度信息，wr = v*/
	    Send_Data.Z_speed = pdu[car_feedback_ang_speed];//小车z轴速度
		Send_Data.Power_Quantity = pdu[BatteryQuantity];
		Send_Data.Power_Voltage = pdu[BatteryVoltage]; //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
		Send_Data.Power_Current = pdu[BatteryCurrent];
		Send_Data.Power_Temperature = pdu[BatteryTemperature];
		 break; 
		
		case Tank_Car:   //履带车
			Send_Data.X_speed = ((MOTOR_A.fltFeedBack_Velocity + MOTOR_B.fltFeedBack_Velocity)/2)*1000; //小车x轴速度
			Send_Data.Y_speed = 0;
			Send_Data.Z_speed = ((MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity)/(Wheel_spacing)*1000);//小车z轴速度
			break; 
		case RC_Car:
			Send_Data.X_speed = (int16_t)(Move_X*1000); //小车x轴速度
			Send_Data.Y_speed = (int16_t)tire_speed;
			Send_Data.Z_speed = (int16_t)(Move_Z*1000);//小车z轴速度 EncPos
			Send_Data.Power_Quantity = (int)mileage >> 16;
			Send_Data.Power_Voltage = (int)mileage;
			Send_Data.Power_Current = (int)EncPos >> 16;
			Send_Data.Power_Temperature = (int)EncPos;
		
		
			break;
		default:
			break;
	}
	
	i = 0;
	uart3_send_data[i++] = Send_Data.Frame_Header; //帧头(固定值)
	uart3_send_data[i++] = Flag_Stop;//电机状态	
	uart3_send_data[i++] = Send_Data.X_speed >>8; //小车x轴速度
	uart3_send_data[i++] = Send_Data.X_speed ;    //小车x轴速度
	uart3_send_data[i++] = Send_Data.Y_speed>>8;  //小车y轴速度
	uart3_send_data[i++] = Send_Data.Y_speed;     //小车y轴速度
	uart3_send_data[i++] = Send_Data.Z_speed >>8; //小车z轴速度
	uart3_send_data[i++] = Send_Data.Z_speed ;    //小车z轴速度	
	uart3_send_data[i++] = Send_Data.Power_Quantity >>8; //电池电量
	uart3_send_data[i++] = Send_Data.Power_Quantity; //电池电量
	uart3_send_data[i++] = Send_Data.Power_Voltage >> 8; //电池电压
	uart3_send_data[i++] = Send_Data.Power_Voltage; //电池电压
	uart3_send_data[i++] = Send_Data.Power_Current >> 8; //电池电流
	uart3_send_data[i++] = Send_Data.Power_Current; //电池电流
	uart3_send_data[i++] = Send_Data.Power_Temperature >> 8; //电池温度
	uart3_send_data[i++] = Send_Data.Power_Temperature; //电池温度

	uart3_send_data[i++] = Send_Data.M1_current >> 8; //电机1电流
	uart3_send_data[i++] = Send_Data.M1_current; //电机1电流
	uart3_send_data[i++] = Send_Data.M2_current >> 8; //电机2电流
	uart3_send_data[i++] = Send_Data.M2_current; //电机2电流
	uart3_send_data[i++] = Send_Data.P19_current >> 8; //19V电源电流
	uart3_send_data[i++] = Send_Data.P19_current; //19V电源电流
	uart3_send_data[i++] = Send_Data.P12_current >> 8; //12V电源电流
	uart3_send_data[i++] = Send_Data.P12_current; //12V电源电流
	uart3_send_data[i++] = Send_Data.P5_current >> 8; //5V电源电流
	uart3_send_data[i++] = Send_Data.P5_current; //5V电源电流
	si = i;
	uart3_send_data[i++]=Check_Sum(&uart3_send_data[0],si,1); //数据校验位计算，模式1是发送数据校验	
	uart3_send_data[i++]=Send_Data.Frame_Tail;//帧尾（固定值）


	DMA_EnableChannel(USARTz_Tx_DMA_Channel, DISABLE);    // 关闭 DMA1 通道2, UART3_TX
	DMA_SetCurrDataCounter(USARTz_Tx_DMA_Channel, i);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
	DMA_EnableChannel(USARTz_Tx_DMA_Channel, ENABLE);    // 开启 DMA1 通道2, UART3_TX	
}


/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：23位为校验位，结果是数组1-22的异或结果；后一个参数为发送或是接收校验位
返 回 值：检验位
**************************************************************************/
uint8_t Check_Sum(uint8_t* d, uint8_t Count_Number, uint8_t Mode)
{
	uint8_t check_sum = 0,k;
	uint8_t* buff = Mode == 0? &Receive_Data.buffer[0]: d;
	//Mode == 0接收数据的校验,Mode == 1发送数据的校验
	for (k = 0;k < Count_Number; k++)
	{
		check_sum = check_sum ^ buff[k];
	}	
	return check_sum;
}



/**************************************************
* 函数功能：	获取到设定的速度和方向值
**************************************************/
void SetReal_Velocity(uint16_t* pdu)
{
	int nTemp = 0;  //临时变量
	int nSwitchSpeed = 0;
	if (ucRcvReady)
	{
		Update_sbus(Uart5_Buffer);
		ucRcvReady = 0;
		remote_off_line_check = 0;
	}
	if (ucRcvReady == 0|| g_ucRos_Flag!=1)
	{
		if (remote_off_line_check < 20)
		{
			remote_off_line_check++;
		}
		else
		{
			g_nDirector = 0;
			g_nVelocity = 0;
			//robot_control.bit.motor_en = 0;
			robot_control.bit.light_ctrl_en = 0;
			g_eControl_Mode = CONTROL_MODE_UNKNOW;
		}
	}
	
	if (g_eControl_Mode == CONTROL_MODE_UART)
	{
		SBUS_CH_Struct* uart_sbus = (SBUS_CH_Struct*) & pdu[virtually_remote_ch1_value];
		memcpy(&tagSBUS_CH, uart_sbus, sizeof(SBUS_CH_Struct));
	}
	SBUSDataRefresh(pdu);

	if (g_eControl_Mode == CONTROL_MODE_UNKNOW||
		g_eControl_Mode == CONTROL_MODE_ROS)
	{
		return;
	}
	//判断SWA是否打开
	rc_ptr = (Remote_Control_struct*)&pdu[turn_off_remote];
	nTemp = tagSBUS_CH.CH7;
	if((Abs_int(nTemp - rc_ptr->turn_off_remote) < 10))
	{
		g_ucRemote_Flag = 0;                   //航模关闭标志位		
		//g_eControl_Mode = CONTROL_MODE_REMOTE; //航模遥控方式选择
		//开关处于关闭状态
		g_nVelocity = 0;
		robot_control.bit.motor_en = 0;
		return;
	}
	 
	else if(Abs_int(nTemp - rc_ptr->turn_on_remote) < 10)
	{
		robot_control.bit.motor_en = 1;
		g_ucRemote_Flag = 1;                   // 航模开启标志位
		g_ucRos_Flag = 0;                      // 航模开启，给与最高权限
		//g_eControl_Mode = CONTROL_MODE_REMOTE; //航模遥控方式选择
		//开关处于打开状态
		nTemp = tagSBUS_CH.CH6;
		
		//此时判断是使用低、中、高速度挡位
		if(Abs_int(nTemp - rc_ptr->speed_level1) < 10)
		{
			//最大速正转返回1，最大速反转返回-1，其余返回的结果只看正负来指示方向，若波动太小视为0
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			//低速档
			if(nSwitchSpeed > 0)
			{
				//正传达到最大速度
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_low;
				else
					g_nVelocity = rc_ptr->speed_low * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;//784=最大值-基准值
			}
			else if(nSwitchSpeed < 0)
			{
				//反转达到最大速度
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
			//中速挡
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			if(nSwitchSpeed > 0)
			{
				//正传达到最大速度
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_middle;
				else
					g_nVelocity = rc_ptr->speed_middle * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else if(nSwitchSpeed < 0)
			{
				//反转达到最大速度
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
			//高速挡
			nSwitchSpeed = Target_Velocity_get(tagSBUS_CH.CH3);
			if(nSwitchSpeed > 0)
			{
				//正传达到最大速度
				if(nSwitchSpeed == 1)
					g_nVelocity = rc_ptr->speed_high;
				else
					g_nVelocity = rc_ptr->speed_high * (float)(tagSBUS_CH.CH3 - rc_ptr->vel_base_value) / 784;
			}
			else if(nSwitchSpeed < 0)
			{
				//反转达到最大速度
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
	
	// 转角获取
	Set_Director();
}



/**************************************************
* 函数功能：	对整型数据取绝对值
**************************************************/
int Abs_int(int nValue)
{
	if(nValue < 0)	return (-nValue);
	else						return nValue;
}




/**************************************************
* 函数功能：	处理电机正转还是反转函数
* 返 回 值：  返回正负分别代表正反转
**************************************************/
int Target_Velocity_get(unsigned short usValue)
{
	int nValue = 0;
	
	//先进行速度限幅设定，并获取速度
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
* 函数功能：	处理电机正转还是反转函数
* 返 回 值：  返回正负分别代表正反转
**************************************************/
int Target_Direct_get(unsigned short usValue)
{
	int nValue = 0;
	
	//先进行角度的限幅设定，并获取角度
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
* 函数功能：	绕Z轴转向的角速度获取
* 返 回 值：  无
**************************************************/
void Set_Director()
{
	
	int nTemp = 0;  //临时变量
	int nSwitchDirect = 0;  //转弯方向的选择
	
	//判断SWA是否打开
	nTemp = tagSBUS_CH.CH7;
	if(Abs_int(nTemp - rc_ptr->turn_off_remote) < 10)
	{
		//开关处于关闭状态
		g_nDirector = 0;
		return;
	}
	
	else if(Abs_int(nTemp - rc_ptr->turn_on_remote) < 10)
	{
		//开关处于打开状态
		nTemp = tagSBUS_CH.CH6;
		if (pdu[car_model] == RC_Car)nTemp = 1023;
		if (pdu[car_model] == FourWheel_Car)nTemp = 240;
		
		//此时判断是使用低、中、高速度挡位
		if(Abs_int(nTemp - rc_ptr->speed_level1) < 10)
		{
			//低速档
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//右传到最大
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_low;
				else
					g_nDirector = rc_ptr->speed_dir_low * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//左转到最大
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
			//中速挡
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//右传到最大
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_middle;
				else
					g_nDirector = rc_ptr->speed_dir_middle * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//左转到最大
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
			//高速挡
			nSwitchDirect = Target_Direct_get(tagSBUS_CH.CH1);
			if(nSwitchDirect > 0)
			{
				//右传到最大
				if(nSwitchDirect == 1)
					g_nDirector = rc_ptr->speed_dir_high;
				else
					g_nDirector = rc_ptr->speed_dir_high * (float)(tagSBUS_CH.CH1 - rc_ptr->dir_base_value) / 784;
			}
			else if(Target_Direct_get(tagSBUS_CH.CH1) < 0)
			{
				//左转到最大
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



/*********************************************
* 函数功能：串口调试打印imu信息

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
/**
 *  @brief  串口1中断控制器配置
 *  @param  无
 *  @retval 无
 *  @note   中断优先级分组全工程只配置一次，在 main 函数最开始进行配置
 *  @note   中断处理函数在 CMSIS/stm32f10x_it.c 中进行处理
 */
void uart1_nvic_config(void)
{
	NVIC_InitType NVIC_InitStruct;
	// 配置串口1的中断控制器
	NVIC_InitStruct.NVIC_IRQChannel = USARTy_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // 子优先级
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
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
	DMA_InitType DMA_InitStruct;

	DMA_DeInit(USARTy_Tx_DMA_Channel);  // DMA1 通道4, UART1_TX
	DMA_DeInit(USARTy_Rx_DMA_Channel);  // DMA1 通道5, UART1_RX

	DMA_StructInit(&DMA_InitStruct);
	// 配置 DMA1 通道4, UART1_TX
	DMA_InitStruct.PeriphAddr = USARTy_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart1_send_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_DST;
	DMA_InitStruct.BufSize = 0;      // 寄存器的内容为0时，无论通道是否开启，都不会发生任何数据传输
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTy_Tx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART1_TX, DMA1, USARTy_Tx_DMA_Channel, ENABLE);

	// 配置 DMA1 通道5, UART1_RX
	DMA_InitStruct.PeriphAddr = USARTy_DR_Base;   // 数据寄存器(USART_DR) 地址偏移：0x04
	DMA_InitStruct.MemAddr = (uint32_t)uart1_recv_data;  // 内存地址
	DMA_InitStruct.Direction = DMA_DIR_PERIPH_SRC;                 // 外设到内存
	DMA_InitStruct.BufSize = USART1_RX_MAXBUFF;
	DMA_InitStruct.PeriphInc = DMA_PERIPH_INC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
	DMA_InitStruct.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
	DMA_InitStruct.MemDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.CircularMode = DMA_MODE_NORMAL;
	DMA_InitStruct.Priority = DMA_PRIORITY_HIGH;
	DMA_InitStruct.Mem2Mem = DMA_M2M_DISABLE;
	DMA_Init(USARTy_Rx_DMA_Channel, &DMA_InitStruct);
	DMA_RequestRemap(DMA1_REMAP_USART1_RX, DMA1, USARTy_Rx_DMA_Channel, ENABLE);



	DMA1_Channel4_nvic_config();
	// 配置 DMA1 通道4, UART1_TX 传输完成中断
	DMA_ConfigInt(USARTy_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);
	/* Get Current Data Counter value before transfer begins */
	//DMA_GetCurrDataCounter(USARTy_Tx_DMA_Channel);



	/* Enable USARTy DMA Rx and TX request */
	USART_EnableDMA(USARTy, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
	DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);     // 开启接收
	DMA_EnableChannel(USARTy_Tx_DMA_Channel, DISABLE);    // 禁止发送
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
	/* Enable GPIO clock */
	RCC_EnableAPB2PeriphClk(USARTy_GPIO_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
	RCC_EnableAHBPeriphClk(USARTy_DMAx_CLK, ENABLE);
	/* Enable USARTy Clock */
	USARTy_APBxClkCmd(USARTy_CLK, ENABLE);
}
/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
	GPIO_InitType GPIO_InitStructure;

	/* Configure USARTy Rx as input floating */
	GPIO_InitStructure.Pin = USARTy_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

	/* Configure USARTy Tx as alternate function push-pull */
	GPIO_InitStructure.Pin = USARTy_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);
}
/**
 *  @brief  串口1初始化
 *  @param  波特率
 *  @retval 无
 *  @note   UART1_TX -> PA9; UART1_RX -> PA10
 */
void USART1_Init(uint32_t baud)
{
	USART_InitType USART_InitStructure;
	/* System Clocks Configuration */
	RCC_Configuration();

	/* Configure the GPIO ports */
	GPIO_Configuration();

	/* USARTy and USARTz configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate = baud;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

	/* Configure USARTy and USARTz */
	USART_Init(USARTy, &USART_InitStructure);

	uart1_nvic_config();    // 中断控制器配置
	uart1_dma_config();     // DMA配置

	USART_ConfigInt(USARTy, USART_INT_IDLEF, ENABLE);  // 使能空闲中断

	/* Enable the USARTy and USARTz */
	USART_Enable(USARTy, ENABLE);
}


/*******************************************************************************
* 函 数 名         : USART1_IRQHandler
* 函数功能		   : USART1中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void USARTy_IRQHandler(void)                	//串口1中断服务程序
{
	if (USART_GetIntStatus(USART1, USART_INT_IDLEF) != RESET)
	{
		USARTy->STS;
		USARTy->DAT;
		DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);     // 暂停接收
		uart1_recv_flag = 1;                // 接收标志置1
		// 统计收到的数据的长度
		uart1_recv_len = USART1_RX_MAXBUFF - DMA_GetCurrDataCounter(USARTy_Rx_DMA_Channel);
	}
}

// DMA1 通道4, UART1_TX 传输完成中断
void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetIntStatus(DMA1_INT_TXC4, DMA1)) // DMA1 通道4, UART1_TX 传输完成
	{
		DMA_ClrIntPendingBit(DMA1_INT_TXC4, DMA1);     // 清除中断
		uart1_send_flag = 1;

		DMA_EnableChannel(USARTy_Tx_DMA_Channel, DISABLE);    // 关闭 DMA1 通道4, UART1_TX

		// 发送完成恢复接收
		//DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
		//DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USART1_RX_MAXBUFF);
		//DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);     // DMA1 通道5, UART1_RX
	}
}
/**************************************************************************

**************************************************************************/
void Pdu_Init()
{
	int i;
	pdu[car_type] = FourWheel_Car;
	pdu[car_model] = FourWheel_Car;
	pdu[car_version] = 0x88;
	pdu[moddbus_485_id] = 1;
	pdu[moddbus_485_baud] = 9;
	pdu[battery_manufacturer] = 0;

	//初始化航模参数
	i = turn_off_remote;
	pdu[i++] = TURN_OFF_REMOTE;
	pdu[i++] = TURN_ON_REMOTE;
	pdu[i++] = VEL_BASE_VALUE;
	pdu[i++] = DIR_BASE_VALUE;
	pdu[i++] = LIMIT_MAX_VAL;
	pdu[i++] = LIMIT_MIN_VAL;
	pdu[i++] = SPEED_LEVEL1;
	pdu[i++] = SPEED_LEVEL2;
	pdu[i++] = SPEED_LEVEL3;
	pdu[i++] = SPEED_LOW;
	pdu[i++] = SPEED_MIDDLE;
	pdu[i++] = SPEED_HIGH;
	pdu[i++] = SPEED_DIR_LOW;
	pdu[i++] = SPEED_DIR_MIDDLE;
	pdu[i++] = SPEED_DIR_HIGH;
	pdu[i++] = LIGHT_BASE;
	pdu[i++] = LIGHT_MAX;
	pdu[i++] = LIGHT_MIN;

	pdu[virtually_remote_ch1_value] = 1023;
	pdu[virtually_remote_ch2_value] = 1023;
	pdu[virtually_remote_ch3_value] = 1023;
	pdu[virtually_remote_ch4_value] = 1023;
	pdu[virtually_remote_ch5_value] = 1023;
	pdu[virtually_remote_ch6_value] = 240;
	pdu[virtually_remote_ch7_value] = 240;
	pdu[virtually_remote_ch8_value] = 240;
	pdu[virtually_remote_ch9_value] = 240;
	pdu[virtually_remote_ch10_value] = 240;
	pdu[virtually_remote_ch11_value] = 1023;
	pdu[virtually_remote_ch12_value] = 1023;
	pdu[virtually_remote_ch13_value] = 1023;
	pdu[virtually_remote_ch14_value] = 1023;
	pdu[virtually_remote_ch15_value] = 1603;
	pdu[virtually_remote_ch16_value] = 1024;

	//初始化电机参数
	int length = motor2_direction - motor1_direction;
	int m_bast_addr;
	uint16_t model = SERVO_WANZE;//< 默认万泽伺服
	for (int j = 0; j < 4; j++) {
		m_bast_addr = j * length;
		i = motor1_direction + m_bast_addr;
		pdu[i++] = 0;					      //节点状态
		pdu[i++] = FourWheer_Radiaus * 10000; //车轮半径，乘以10000后
		pdu[i++] = REDUCTION_RATE * 100;	  //减速比，乘以100后
		pdu[i++] = j + 1;					  //CAN ID
		pdu[i++] = 1000;					  //CAN波特率，除以100后
		pdu[i++] = 0;						  //心跳  
		pdu[i++] = model;					  //伺服厂家型号
		pdu[i++] = 500;						  //编码器精度
		pdu[i++] = 0;
		pdu[i++] = 0;						  //目标转速
		pdu[i++] = 0;
		pdu[i++] = 0;						  //目标位置
	}
	pdu[motor1_CAN_id] = 6;
	pdu[motor2_CAN_id] = 2;	
	pdu[motor3_CAN_id] = 5;
	pdu[motor4_CAN_id] = 1;	
	
	//初始化小车速度限幅参数
	pdu[car_max_lin_speed] = 802;
	pdu[car_min_lin_speed] = -802;
	pdu[car_max_ang_speed] = 801;
	pdu[car_min_ang_speed] = -801;
	pdu[car_tar_lin_speed] = 0;
	pdu[car_tar_ang_speed] = 0;
	pdu[motor1_radius] = FourWheer_Radiaus * 10000;
	pdu[motor1_reduction_ratio] = REDUCTION_RATE * 100;
	FourWheer_Perimeter = 2 * PI * FourWheer_Radiaus;//< 车轮周长
	FourWheer_Conversion = FourWheer_Perimeter / 60 / REDUCTION_RATE;
	VelocityToRpmConversion = (60 * REDUCTION_RATE) / FourWheer_Perimeter;
	AngularVelocityConversion = DIRECTOR_BASE * (PI / 4);
	pdu[car_default_mode] = g_eControl_Mode;
	pdu[wheel_distance] = (uint16_t)(Wheel_spacing * 10000);
	pdu[axles_distance] = (uint16_t)(Axle_spacing * 10000);
	pdu[motor_num] = Motor_Number=4;
	pdu[car_mode] = CONTROL_MODE_REMOTE;

	pdu[Middle_battery_threshold] = 40;
	pdu[Low_battery_threshold] = 25;

}

void modbus_task_init(void)
{
	UCHAR mySlaveAddress = 0x01;//< 从机地址
	pdu = getPDUData();
		//初始化pdu数组
	eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
	MyFLASH_ReadByte(FINAL_PAGE_ADDRESS,pdu , MB_RTU_DATA_MAX_SIZE);
	if (pdu[car_type] == 0xFFFF)
	{//< 芯片首次初始化
		eMBInit(MB_RTU, mySlaveAddress, 3, 115200, MB_PAR_NONE);
		Pdu_Init();
		MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		pdu[para_save] = 10;
	}
	else pdu[para_save] = 0;
}

void ModBUS_task(void* pvParameters)
{
	MBModify modify;
	/* Enable the Modbus Protocol Stack. */
	eMBEnable();

	while (1)
	{
		rt_thread_delay(10);   //< 1ms
		if (uart1_recv_flag == 1)
		{
			LedBlink(LED1_PORT, RUN1);
			uart1_recv_flag = 0;
			pxMBFrameCBByteReceived();
		}
		eMBPoll(&modify);
		if (uart1_send_flag == 2)
		{//< 回复帧
			uart1_send_flag = 0;
			DMA_EnableChannel(DMA1_CH4, DISABLE);    // 关闭 DMA1 通道4, UART1_TX
			DMA_SetCurrDataCounter(DMA1_CH4, uart1_send_len);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
			DMA_EnableChannel(DMA1_CH4, ENABLE);    // 开启 DMA1 通道4, UART1_TX
		// 发送完成恢复接收
			DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
			DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USART1_RX_MAXBUFF);
			DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);     // DMA1 通道5, UART1_RX
			Modbus_Respond(&modify);
		}
		if (pdu[para_save]==1)
		{//< 保存当前所有参数
			pdu[para_save] = 0;
			MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		}
		else if (pdu[para_save] == 2)
		{//< 恢复出厂设置
			pdu[para_save] = 0;
			eMBInit(MB_RTU, 1, 3, 115200, MB_PAR_NONE);
			Pdu_Init();
			MyFLASH_WriteWord(FINAL_PAGE_ADDRESS, pdu, MB_RTU_DATA_MAX_SIZE);
		}


		if (pdu[software_reset]==0xa5)
		{
			pdu[software_reset] = 0;
			Soft_Reset();
		}
		else if (pdu[software_reset] == 0x5a)
		{
			pdu[software_reset] = 0;
			Jump_To_BOOT();
		}
		if (pdu[error_get_and_clear] == 1)
		{
			pdu[error_get_and_clear] = 0;
			error_code = 0;
		}

	}
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
