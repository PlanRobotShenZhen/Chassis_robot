#include "user_can.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"
#include "string.h"
#include "usartx.h"
#include "motor_data.h"
#include <stdlib.h>
#include "rtthread.h"
#include "n32g45x.h"
#include "485_address.h"
#include "rtthread.h"
#include "robot_select_init.h"
#include "balance.h"


static struct SdoFrame* sdo_head = NULL;

CanRxMessage M1T0Message, M2T0Message, M3T0Message, M4T0Message, M5T0Message, 
			 M6T0Message, M7T0Message, M8T0Message, M9T0Message, M10T0Message,
			 M1T1Message, M2T1Message, M3T1Message, M4T1Message, M5T1Message, 
			 M6T1Message, M7T1Message, M8T1Message, M9T1Message, M10T1Message, 
			 M1T2Message, M2T2Message, M3T2Message, M4T2Message, M5T2Message, 
			 M6T2Message, M7T2Message, M8T2Message, M9T2Message, M10T2Message; 

bool M1T0Message_FLAG, M2T0Message_FLAG, M3T0Message_FLAG, M4T0Message_FLAG, M5T0Message_FLAG, 
	 M6T0Message_FLAG, M7T0Message_FLAG, M8T0Message_FLAG, M9T0Message_FLAG, M10T0Message_FLAG,
	 M1T1Message_FLAG, M2T1Message_FLAG, M3T1Message_FLAG, M4T1Message_FLAG, M5T1Message_FLAG, 
	 M6T1Message_FLAG, M7T1Message_FLAG, M8T1Message_FLAG, M9T1Message_FLAG, M10T1Message_FLAG,
	 M1T2Message_FLAG, M2T2Message_FLAG, M3T2Message_FLAG, M4T2Message_FLAG, M5T2Message_FLAG, 
	 M6T2Message_FLAG, M7T2Message_FLAG, M8T2Message_FLAG, M9T2Message_FLAG, M10T2Message_FLAG;


MOTOR_TPDO mtd[MAX_MOTOR_NUMBER];//< 发送pdo
MOTOR_RPDO mrd[MAX_MOTOR_NUMBER];//< 接收pdo

//超过时长没产生心跳包1000
int g_nHeart_Time_LB = 0;
int g_nHeart_Time_LF = 0;
int g_nHeart_Time_RF = 0;
int g_nHeart_Time_RB = 0;
int send_number = 1;

/**
 * @brief  Configures CAN GPIOs
 */
void CAN_GPIO_Config(void)
{
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStruct(&GPIO_InitStructure);
	RCC_EnableAPB2PeriphClk(CANb_CLK, ENABLE);//CANa与CANb同时钟
	/* Configures CAN1 IOs */
#if(CARMODE != Diff)
	{
		RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
		/* Remap CAN1 GPIOs */
		GPIO_ConfigPinRemap(GPIO_RMP2_CAN1, ENABLE);
		/* Configure CAN1 RX pin */
		GPIO_InitStructure.Pin = CANa_RxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitPeripheral(CANa_GPIO, &GPIO_InitStructure);
		/* Configure CAN1 TX pin */
		GPIO_InitStructure.Pin = CANa_TxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitPeripheral(CANa_GPIO, &GPIO_InitStructure);
	}
#endif
	/* Configure CAN2 RX pin */
	GPIO_InitStructure.Pin = CANb_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitPeripheral(CANb_GPIO, &GPIO_InitStructure);
	/* Configure CAN2 TX pin */
	GPIO_InitStructure.Pin = CANb_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(CANb_GPIO, &GPIO_InitStructure);
	//GPIO_ConfigPinRemap(GPIO_RMP3_CAN2, ENABLE);
}
/**
 * @brief  Configures CAN Filer.
 */
void CAN1_Filter_Init(void)
{
    CAN_FilterInitType CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_LowId          = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_HighId     = CAN_STD_ID_H_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_LowId      = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN1_InitFilter(&CAN_FilterInitStructure);
    CAN_INTConfig(CAN1, CAN_INT_FMP0, ENABLE);
}


/**
 * @brief  Configures CAN Filer.
 */
void CAN2_Filter_Init(void)
{
    CAN_FilterInitType CAN_FilterInitStructure;
    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdMaskMode;
    CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_32bitScale;
    CAN_FilterInitStructure.Filter_HighId         = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_LowId          = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_HighId     = CAN_STD_ID_H_MASK_DONT_CARE;
    CAN_FilterInitStructure.FilterMask_LowId      = CAN_STD_ID_L_MASK_DONT_CARE;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN2_InitFilter(&CAN_FilterInitStructure);
    CAN_INTConfig(CAN2, CAN_INT_FMP0, ENABLE);
}
/**
 * @brief  Configures the NVIC for CAN.
 */
void CAN_NVIC_Config(void)
{
	NVIC_InitType NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x2;
	NVIC_Init(&NVIC_InitStructure);
}
/**
 * @brief  Configures CAN1 and CAN2.
 * @param CAN_BaudRate 10Kbit/s ~ 1Mbit/s
 */
void CAN_Config(uint16_t baud)
{
	CAN_InitType CAN_InitStructure;
	/* Configure CAN1 and CAN2 */
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1 | RCC_APB1_PERIPH_CAN2, ENABLE);
	/* CAN1 and CAN2 register init */
	CAN_DeInit(CAN1);
	CAN_DeInit(CAN2);
	/* Struct init*/
	CAN_InitStruct(&CAN_InitStructure);
	/* CAN1 and CAN2  cell init */
	CAN_InitStructure.TTCM = DISABLE;
	CAN_InitStructure.ABOM = ENABLE;
	CAN_InitStructure.AWKUM = DISABLE;
	CAN_InitStructure.NART = ENABLE;
	CAN_InitStructure.RFLM = DISABLE;
	CAN_InitStructure.TXFP = DISABLE;
	CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
	switch (baud)
	{
	case 1://< 500K
		CAN_InitStructure.RSJW = CAN_RSJW_1tq;
		CAN_InitStructure.TBS1 = CAN_TBS1_5tq;
		CAN_InitStructure.TBS2 = CAN_TBS2_2tq;
		CAN_InitStructure.BaudRatePrescaler = 9;
		break;
	default://< 1M
		CAN_InitStructure.RSJW = CAN_RSJW_2tq;
		CAN_InitStructure.TBS1 = CAN_TBS1_14tq;
		CAN_InitStructure.TBS2 = CAN_TBS2_3tq;
		CAN_InitStructure.BaudRatePrescaler = 2;
		break;
	}
	/*Initializes the CAN1  and CAN2 */
	
#if (CARMODE != Diff)
	{
		CAN_Init(CAN1, &CAN_InitStructure);
		CAN1_Filter_Init();
	}
#endif
	CAN_Init(CAN2, &CAN_InitStructure);

	CAN2_Filter_Init();
}

/**********************************************************
* 函数功能： stm32底层的can通信协议初始化。
* 参数：     无
* 说明：     无
**********************************************************/
void Can_Driver_Init(uint8_t baud)
{
	CAN_NVIC_Config();
	/* Configures CAN IOs */
	CAN_GPIO_Config();
	/* Configures CAN */
	CAN_Config(baud);
}



uint16_t DevCANOpen_send_sdo(uint16_t id, uint8_t Data_Size, uint8_t* sdo_data)
{
	uint16_t ret = 0;
	CanTxMessage pTxMessage;
	int nIndex = 0;
	int nReturn = -1;
	int errCount = 0;

	pTxMessage.DLC = Data_Size;
	pTxMessage.IDE = CAN_Standard_Id;
	pTxMessage.RTR = CAN_RTRQ_Data;
	pTxMessage.StdId = id;
	for (nIndex = 0; nIndex < Data_Size; nIndex++)
	{
		pTxMessage.Data[nIndex] = sdo_data[nIndex];
	}
	do
	{
		nReturn = CAN_TransmitMessage(CAN1, &pTxMessage); //发送成功，返回0~2(邮箱号)，失败返回0x04
		errCount++;
		if (errCount > 10)
		{
			ret = 1;
			break;
		}
	} while (nReturn == 0x04);
	return ret;
}


/*保存参数配置至EEPROM，未调用*/
uint8_t Save_EEPROM(uint8_t ID)
{
	uint8_t Data[8] = {0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	return DevCANOpen_send_sdo(0x600 + ID, 0x08, Data);
}


/*恢复出厂配置，未调用*/
uint8_t Reset_Driver(uint8_t ID)
{
	uint8_t Data[8] = {0x2B, 0x09, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	return DevCANOpen_send_sdo(0x600 + ID, 0x08, Data);
}

struct SdoFrame* SdoFramCereat(void) 
{
	struct SdoFrame* new_sdo_frame = NULL;
	struct SdoFrame* next_sdo_frame;
	//rt_malloc
	new_sdo_frame = (struct SdoFrame*)rt_malloc(sizeof(struct SdoFrame));
	if (new_sdo_frame != NULL)
	{
		if (sdo_head == NULL)
			
		{
			sdo_head = new_sdo_frame;//创建一个头结点
			sdo_head->next = NULL;
		}
		else
		{
			next_sdo_frame = sdo_head;
			while (next_sdo_frame->next != NULL)
			{
				next_sdo_frame = next_sdo_frame->next;
			}
			next_sdo_frame->next = new_sdo_frame;
			new_sdo_frame->next = NULL;
		}
		return new_sdo_frame;
	}
	return NULL;
}

/**********************************************************
 * 函数功能： ZLAC_PDO事件配置。
 * 参数：     ID代表节点地址。
 * 说明：     配置PDO。
 **********************************************************/
void ZLAC8015_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)// 中菱伺服
{
	uint8_t v[7];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//定时器50ms
	v[3] = 0xFF;		//速度模式地址
	v[4] = 0x20;		//4字节		
	v[5] = 0x29;		//映射地址
	v[6] = 0x20;		//映射地址

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//位置模式地址
	}else if(mode == torque_mode){
		v[3] = 0x71;	//转矩模式地址
		v[4] = 0x10;	//2字节
		v[5] = 0x77;
		v[6] = 0x60;
	}

	uint8_t Init_sdo[][8] = {// 主站(单片机)，恢复从站的初始值
		// 读参数映射（TPDO0）
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  	//失能Tpdo0
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    	//定时器触发时间 20/50ms(刷新率为50/20Hz)	
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  	//清除映射
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  	//6041  状态字
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x64,0x60},  	//6064  当前位置
		{0x23,0x00,0x1A,0x03,0x10,0x00,0x26,0x20},  	//2026  PCB温度
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00}, 		//设置PDO的COB-ID
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    	//定时器触发
		{0x2F,0x00,0x1A,0x00,0x03,0x00,0x00,0x00},    	//映射对象子索引个数
		// 读参数映射（TPDO1）
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80},  	//失能Tpdo1
		{0x2B,0x01,0x18,0x05,0x32,0x00,0x00,0x00},    	//定时器触发时间 50ms(刷新率为20Hz)
		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  	//清除映射
		{0x23,0x01,0x1A,0x01,0x20,0x00,0x6C,0x60},  	//606C  速度反馈	
		{0x23,0x01,0x1A,0x02,0x10,0x00,v[5],v[6]},  	//2029/6077 母线电压/转矩反馈				
		{0x23,0x02,0x1A,0x01,0x10,0x00,0x3F,0x60},  	//603F  读取故障码					
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, 		//设置PDO的COB-ID
		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    	//定时器触发
		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    	//映射对象子索引个数				
		// 写参数映射（RPDO0）
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  	//失能Rpdo0
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    	//②Hex1400_02，传输形式：0xFE：事件触发；0xFF：定时器触发
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  	//清除映射
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  	//6040 控制字
    	{0x23,0x00,0x16,0x02,v[4],0x00,v[3],0x60},		//运动模式
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},   	//映射对象子索引个数
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  	//设置PDO的COB-ID	
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
	if(mode == position_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00},  //设置位置模式	
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//设置加速时间50ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//设置减速时间50ms
			{0x23,0x81,0x60,0x00,0xFC,0x08,0x00,0x00},  //设置最大速度2300r/min	
			{0x2B,0x5D,0x60,0x00,0x03,0x00,0x00,0x00},  //HALT急停	
			{0x2B,0x17,0x20,0x00,0x70,0x17,0x00,0x00},  //超差报警阈值6000*10counts
		};																																//电机PDO配置															//反转电平	
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}else if(mode == speed_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},  //设置速度模式	
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//设置加速时间100ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//设置减速时间100ms
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));	
		Driver_JT_Inv(id);			//反转电平	
	}else if(mode == torque_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x04,0x00,0x00,0x00},  //设置力矩模式
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//设置加速时间100ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//设置减速时间100ms
			{0x23,0x87,0x60,0x00,0x10,0x27,0x00,0x00},  //转矩斜率10000	
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}
}

/**********************************************************
 * 函数功能： 驱动输入端子电平反转。
 * 参数：     无。
 * 说明：     电源板默认设计低电平为有效电平，驱动急停是高电平有效，为了适配电源板，需要反转电平。
 **********************************************************/
void Driver_JT_Inv(uint8_t id)
{
	uint8_t Init_sdo[][8] = {0x2B,0x30,0x20,0x01,0x01,0x00,0x00,0x00}; // 反转输入端子X0与X1的电平，即给Bit0、1置1
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
}


/**********************************************************
 * 函数功能： RPDO0事件触发。
 * 参数：     ID代表节点地址。
 * 说明：     RPDO0映射0x6040（控制字）；
 *            RPDO0-COB-ID:0x200 + ID
 **********************************************************/
void ZLAC8015D_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)
{
	uint8_t Init_sdo[][8] = {// 主站(单片机)，恢复从站的初始值 	
		{0x2B,0x0F,0x20,0x00,0x01,0x00,0x00,0x00},	//设置同步控制
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//设置速度模式
		//// 中菱伺服
		{0x23,0x83,0x60,0x01,al,ah,0x00,0x00},  	// 6083 01左电机设置加速时间100ms
		{0x23,0x83,0x60,0x02,al,ah,0x00,0x00},  	// 6083 02右电机设置加速时间100ms
		{0x23,0x84,0x60,0x01,dl,dh,0x00,0x00},  	// 6084 01左电机设置减速时间100ms
		{0x23,0x84,0x60,0x02,dl,dh,0x00,0x00},  	// 6084 02右电机设置减速时间100ms
		// 读参数映射（TPDO0）
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // 消去个数: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 状态字
		{0x23,0x00,0x1A,0x02,0x20,0x03,0x6C,0x60},  // 606C 实时反馈速度
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},  //②Hex1800_02，传输形式：0xFE：事件触发；0xFF：定时器触发 1个有效数据，设置TPDO1的传输类型，SYNC 
		{0x2B,0x00,0x18,0x05,0x32,0x00,0x00,0x00},  //定时器触发时间50ms(刷新率为20Hz)
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},  //⑥Hex1A00_0，启用【映射参数】 【实际映射多少组】 
		// 写参数映射（RPDO0）
		{0x2F,0x01,0x16,0x00,0x00,0x00,0x00,0x00},  // 清除映射: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x01,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 控制字
		{0x23,0x01,0x16,0x02,0x20,0x03,0xFF,0x60},  // 60FF 目标速度
		{0x2F,0x01,0x16,0x00,0x02,0x00,0x00,0x00},  //映射对象子索引个数	⑥Hex1600_0，启用【映射参数】 【实际映射多少组】 
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
}

/**********************************************************
 * 函数功能： WZ_PDO事件配置。
 * 参数：     ID代表节点地址。
 * 说明：     配置PDO。
 **********************************************************/
void WANZER_PDO_Config(uint8_t id, uint8_t mode)// 万泽伺服
{
	uint8_t v[4];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//定时器50ms
	v[3] = 0xFF;		//速度模式地址

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//位置模式地址
	}
	uint8_t Init_sdo[][8]={ // 主站(单片机)，恢复从站的初始值 
		// 读参数映射（TPDO0）
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  		//失能Tpdo0
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    		//定时器触发时间 20/50ms(刷新率为50/20Hz)	
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  			//清除映射
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  			//6041  状态字
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x64,0x60},  			//6064  当前位置
		{0x23,0x00,0x1A,0x03,0x10,0x04,0x04,0x20},  			//2004  PCB温度	
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00}, 		//设置PDO的COB-ID
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    		//定时器触发
		{0x2F,0x00,0x1A,0x00,0x03,0x00,0x00,0x00},    		//映射对象子索引个数
		// 读参数映射（TPDO1）
//		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80},  		//失能Tpdo1
//		{0x2B,0x01,0x18,0x05,v[2],0x00,0x00,0x00},    		//定时器触发时间 50ms(刷新率为20Hz)
//		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  			//清除映射
//		{0x23,0x01,0x1A,0x01,0x20,0x00,0x6C,0x60},  			//606C  速度反馈
//		{0x23,0x01,0x1A,0x02,0x10,0x02,0x04,0x20},  			//2004  母线电压
//		{0x23,0x01,0x1A,0x03,0x10,0x03,0x04,0x20},  			//2004  母线电流	
//		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, 		//设置PDO的COB-ID
//		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    		//定时器触发
//		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    		//映射对象子索引个数	
		// 写参数映射（RPDO0）
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  				//失能Rpdo0
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    		//②Hex1400_02，传输形式：0xFE：事件触发；0xFF：定时器触发
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  			//清除映射
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  			//6040 控制字
    	{0x23,0x00,0x16,0x02,0x20,0x00,v[3],0x60},  			//60FF/7A 速度、位置模式
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},   		  //映射对象子索引个数
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  				// 设置PDO的COB-ID	
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
	if(mode == position_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00},  //设置位置模式	
			{0x23,0x65,0x60,0x00,0x40,0x0D,0x03,0x00},  //超差报警阈值20*10000counts
			{0x23,0x66,0x60,0x00,0x88,0x13,0x00,0x00},  //超差报警阈值 5000ms
			{0x23,0x81,0x60,0x00,0xC0,0x27,0x09,0x00},  // 6081 写入参数轮廓位置模式最大规划速度 600000			
			{0x23,0x83,0x60,0x00,0xD0,0x09,0x00,0x00},  // 6083 写入参数轮廓位置模式加速度 2512
			{0x23,0x84,0x60,0x00,0xD0,0x09,0x00,0x00},  // 6084 写入参数轮廓位置模式减速度 2512
			{0x23,0xa4,0x60,0x01,0xD0,0x09,0x00,0x00},  // 60a4 01 写入参数轮廓加加速度 2512
			{0x23,0xa4,0x60,0x02,0xD0,0x09,0x00,0x00},  // 60a4 02 写入参数轮廓加加速度 2512	
		};																																//电机PDO配置															//反转电平	
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}else if(mode == speed_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},  //设置速度模式	
			{0x23,0x83,0x60,0x00,0x10,0x27,0x00,0x00},  // 6083 写入参数轮廓位置模式加速度 10000
			{0x23,0x84,0x60,0x00,0x10,0x27,0x00,0x00},  // 6084 写入参数轮廓位置模式减速度 10000
			{0x23,0xa4,0x60,0x01,0xA0,0x86,0x01,0x00},  // 60a4 01 写入参数轮廓加加速度 100000
			{0x23,0xa4,0x60,0x02,0xA0,0x86,0x01,0x00},  // 60a4 02 写入参数轮廓加加速度 100000
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));	
	}
}

/**********************************************************
 * 函数功能： PL_PDO事件配置。
 * 参数：     ID代表节点地址。
 * 说明：     配置PDO。
 **********************************************************/
void PLAN_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)// 普蓝伺服
{
	uint8_t v[5];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//定时器50ms
	v[3] = 0xFF;		//速度模式地址
	v[4] = 0x20;		//4字节		

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//位置模式地址
	}else if(mode == torque_mode){
		v[3] = 0x71;	//转矩模式地址
		v[4] = 0x10;	//2字节
	}
	uint8_t Init_sdo[][8]={ // 主站(单片机)，恢复从站的初始值 	
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  // 失能Rpdo： 发送sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},  //②Hex1400_02，传输形式：0xFE：事件触发；0xFF：定时器触发
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // 清除映射: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 控制字
		{0x23,0x00,0x16,0x02,v[4],0x00,v[3],0x60},  // 60FF 目标速度
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //映射对象子索引个数	⑥Hex1600_0，启用【映射参数】 【实际映射多少组】 
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		// 读参数映射TPDO0
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  // 失能Tpdo： 发送sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    //定时器触发时间 50ms
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // 消去个数: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 状态字
		{0x23,0x00,0x1A,0x02,v[4],0x00,0x6C,0x60},  // 606C 实时反馈速度
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//设置速度模式
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //②Hex1800_02，传输形式：0xFE：事件触发；0xFF：定时器触发 1个有效数据，设置TPDO1的传输类型，SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //⑥Hex1A00_0，启用【映射参数】 【实际映射多少组】
		// 读参数映射（TPDO1）
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80}, //失能Tpdo1
		{0x2B,0x01,0x18,0x05,v[2],0x00,0x00,0x00},    	//定时器触发时间 100ms(刷新率为10Hz)
		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  	//清除映射
		{0x23,0x01,0x1A,0x01,0x10,0x0d,0x00,0x20},  	//2000 0d RPM		
		{0x23,0x01,0x1A,0x02,0x10,0x00,0x78,0x60},  	//6078 00 电流反馈	
		{0x23,0x01,0x1A,0x03,v[4],0x00,0x64,0x60},  	//606C 00 位置反馈	
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, //设置PDO的COB-ID
		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    //②Hex1801_02，传输形式：0xFE：事件触发；0xFF：定时器触发 1个有效数据，设置TPDO1的传输类型，SYNC 
		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    //⑥Hex1A01_0，启用【映射参数】 【实际映射多少组】 
		// 读参数映射（TPDO2）
		{0x23,0x02,0x18,0x01,v[0],0x03,0x00,0x80}, //失能Tpdo2
		{0x2B,0x02,0x18,0x05,v[2],0x00,0x00,0x00},    	//定时器触发时间 100ms(刷新率为10Hz)
		{0x2F,0x02,0x1A,0x00,0x00,0x00,0x00,0x00},  	//清除映射
		{0x23,0x02,0x1A,0x01,0x10,0x00,0x3f,0x60},  	//603f 00 Errorcode		
		{0x23,0x02,0x1A,0x02,0x08,0x00,0x61,0x60},  	//6061 00 模式
		{0x23,0x02,0x18,0x01,v[0],0x03,0x00,0x00}, //设置PDO的COB-ID
		{0x2F,0x02,0x18,0x02,0xFF,0x00,0x00,0x00},    
		{0x2F,0x02,0x1A,0x00,0x02,0x00,0x00,0x00},    
	};
		Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));	
}

/**********************************************************
* 函数功能： NMT节点状态设置，对节点使能、状态切换。
* 参数：     ID代表节点地址；data0为功能选择，data1为节点地址。
* 说明：     激活驱动PDO传输
**********************************************************/
void NMT_Control(const uint8_t Data0, const uint8_t ID)
{
	struct SdoFrame* new_sdo_frame;
	new_sdo_frame = SdoFrameCreate();
	if (new_sdo_frame != NULL){
		new_sdo_frame->ID = 0x000;
		new_sdo_frame->mode = 1;
		new_sdo_frame->data[0] = Data0;
		new_sdo_frame->data[1] = ID;
	}  
}

struct PointFrame
{
	uint32_t data[10];
	struct PointFrame* next;
};
static struct PointFrame* point_data_head = NULL;

struct PointFrame* PointFramCereat(void)
{
	struct PointFrame* new_point_frame = NULL;
	struct PointFrame* next_sdo_frame;
	//rt_malloc
	new_point_frame = (struct PointFrame*)rt_malloc(sizeof(struct PointFrame));
	if (new_point_frame != NULL)
	{
		if (point_data_head == NULL)

		{
			point_data_head = new_point_frame;//创建一个头结点
			point_data_head->next = NULL;
		}
		else
		{
			next_sdo_frame = point_data_head;
			while (next_sdo_frame->next != NULL)
			{
				next_sdo_frame = next_sdo_frame->next;
			}
			next_sdo_frame->next = new_point_frame;
			new_point_frame->next = NULL;
		}
		return new_point_frame;
	}
	return NULL;
}

/**************************************************************************
函数功能：CAN PDO任务
入口参数：无
返回  值：无
**************************************************************************/
void Can_task(void* pvParameters)
{
	while (1){
		rt_thread_delay(20);   //< 2ms
		CAN_SDOSend(CAN1);
		CAN_PDOSend(pdu[motor_number], CAN1);
	}

}

/**************************************************************************
函数功能：电机CAN之RPDO传输任务
入口参数：无 
返回  值：无
**************************************************************************/
void CAN_SDOSend(CAN_Module* CANx)
{
	if (sdo_head != NULL){
		CanTxMessage pTxMessage;
		int nIndex = 0;
		int nReturn = -1;
		uint8_t Data_Size = sdo_head->mode == 0 ? 8 : 2;
		pTxMessage.DLC = Data_Size;
		pTxMessage.IDE = CAN_Standard_Id;
		pTxMessage.RTR = CAN_RTRQ_Data;
		pTxMessage.StdId = sdo_head->ID;
		for (nIndex = 0; nIndex < Data_Size; nIndex++){
			pTxMessage.Data[nIndex] = sdo_head->data[nIndex];
		}
		nIndex = 0;
		do{
			nIndex++;
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //发送成功，返回0~2(邮箱号)，失败返回0x04
		} while (nReturn == 0x04);
		struct SdoFrame* next_sdo = sdo_head->next;
		rt_free(sdo_head);
		sdo_head = next_sdo;
	}
}

/**********************************************************
 * 函数功能： CANPDO帧发送函数。
 * 参数：     number代表节点数量。
 * 说明：     该函数必须在PDO配置完成之后运行。
 * 返回值：   发送成功数量
 **********************************************************/
void CAN_PDOSend(uint32_t number, CAN_Module* CANx)
{
	MOTOR_RPDO* rpdo;
	int nReturn = 0x04;
	static int send_number = 0;
	int target_pos_pulse;
	CanTxMessage pTxMessage;
	pTxMessage.IDE = CAN_Standard_Id;
	pTxMessage.RTR = CAN_RTRQ_Data;
	pTxMessage.DLC = 6;//< 

	do{
		rpdo = &mrd[send_number];
		uint16_t x_id = pdu[motor1_CAN_id + send_number * pdu[rw_motor_gap]];
		uint16_t x_type = pdu[motor1_type + send_number * pdu[ro_motor_gap]];
		uint16_t x_sport_mode = pdu[motor1_sport_mode + send_number * pdu[ro_motor_gap]];
			pTxMessage.StdId = RPDO0_ID + x_id;
			pTxMessage.Data[0] = rpdo->d.ctrl.cw & 0xFF;      // 写入控制字
			pTxMessage.Data[1] = (rpdo->d.ctrl.cw >> 8) & 0xFF;
			switch (x_type){
				case servo_zlac:
					pTxMessage.Data[2] = rpdo->d.target_pos_vel & 0xFF;       // 低八位写入 Data[2]
					pTxMessage.Data[3] = (rpdo->d.target_pos_vel >> 8) & 0xFF; // 
					pTxMessage.Data[4] = (rpdo->d.target_pos_vel >> 16) & 0xFF; // 
					pTxMessage.Data[5] = (rpdo->d.target_pos_vel >> 24) & 0xFF; // 高八位写入 Data[5]
					break;
				case servo_wanze:
					target_pos_pulse = rpdo->d.target_pos_vel;
					switch (x_sport_mode){
						case speed_mode:
							target_pos_pulse = rpdo->d.target_pos_vel * ENCODER_LINES / MINTOSEC; //默认单位不同，需要将单位从rpm转换成pulse
							break;
						case position_mode://默认单位相同
							break;
					}
					pTxMessage.Data[2] = target_pos_pulse & 0xFF;       // 低八位写入 Data[2]
					pTxMessage.Data[3] = (target_pos_pulse >> 8) & 0xFF; // 
					pTxMessage.Data[4] = (target_pos_pulse >> 16) & 0xFF; // 
					pTxMessage.Data[5] = (target_pos_pulse >> 24) & 0xFF; // 高八位写入 Data[5]
					break;
				case servo_zlacd:
					pTxMessage.Data[2] = mrd[0].d.target_pos_vel & 0xFF;       	// 左电机目标速度，低八位写入 Data[2]
					pTxMessage.Data[3] = (mrd[0].d.target_pos_vel >> 8) & 0xFF; // 
					pTxMessage.Data[4] = mrd[1].d.target_pos_vel & 0xFF; 		// 右电机目标速度，低八位写入 Data[2]
					pTxMessage.Data[5] = (mrd[1].d.target_pos_vel >> 8) & 0xFF; // 高八位写入 Data[5]
					break;
				case servo_plan:
					/* code */
					break;
			}
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //若发送失败，返回0x04
		if (nReturn != 0x04){
			send_number++;//< 发送下一个
			if (send_number >= number){
				send_number = 0; 
			}
		}else{			
		break;
		}
	} while (1);
}

/**********************************************************
 * 函数功能： 将消息加入到sdo链表
 * 参数：     无
 * 返回值：   无
 **********************************************************/
void Add_Sdo_Linked_List(uint8_t id, uint8_t Init_sdo[][8], int sdo_count)
{
    struct SdoFrame *new_sdo_frame;
    for (int i = 0; i < sdo_count; i++){
        new_sdo_frame = SdoFrameCreate();
        if (new_sdo_frame != NULL){
            new_sdo_frame->ID = SDO_M_ID + id;
            new_sdo_frame->mode = 0;
            for (int k = 0; k < 8; k++){
                new_sdo_frame->data[k] = Init_sdo[i][k];
			}
        }
    }
}

/**********************************************************
 * 函数功能： 创建一个 SDO 帧
 * 参数：     无
 * 说明：     该函数用于动态创建一个 SDO 帧节点，并将其添加到 SDO 链表中。
 * 返回值：   如果成功创建并添加到链表，返回指向新创建的 SDO 帧节点的指针，否则返回 NULL。
 **********************************************************/
struct SdoFrame* SdoFrameCreate(void) 
{
	struct SdoFrame* new_sdo_frame = NULL;
	struct SdoFrame* next_sdo_frame;
	new_sdo_frame = (struct SdoFrame*)rt_malloc(sizeof(struct SdoFrame));
	if (new_sdo_frame != NULL)
	{
		if (sdo_head == NULL)
		{
			sdo_head = new_sdo_frame;//创建一个头结点
			sdo_head->next = NULL;
		}
		else
		{
			next_sdo_frame = sdo_head;
			while (next_sdo_frame->next != NULL)
			{
				next_sdo_frame = next_sdo_frame->next;
			}
			next_sdo_frame->next = new_sdo_frame;
			new_sdo_frame->next = NULL;
		}
		return new_sdo_frame;
	}
	return NULL;
}

/**********************************************************
 * 函数功能： 判断 SDO 队列是否为空
 * 参数：     无
 * 说明：     该函数用于检查 SDO 队列是否为空，通过检查头指针是否为空来判断。
 * 返回值：   1 表示 SDO 队列为空，0 表示 SDO 队列非空
 **********************************************************/
uint8_t IsSdoEmpty(void)
{
	return sdo_head == NULL;
}
