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

struct SdoFrame
{
	uint16_t ID;
	uint8_t mode;//<0:sdo,1:nmt
	uint8_t data[8];
	struct SdoFrame* next;
};
static struct SdoFrame* sdo_head = NULL;

//超过时长没产生心跳包1000
int g_nHeart_Time_LB = 0;
int g_nHeart_Time_LF = 0;
int g_nHeart_Time_RF = 0;
int g_nHeart_Time_RB = 0;
int send_number = 1;

uint8_t IsSdoEmpty(void)
{
	return sdo_head == NULL;
}
/**
 * @brief  Configures CAN GPIOs
 */
void CAN_GPIO_Config(void)
{
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStruct(&GPIO_InitStructure);
	/* Configures CAN1 IOs */
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOB, ENABLE);
	/* Configure CAN1 RX pin */
	GPIO_InitStructure.Pin = CANa_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitPeripheral(CANa_GPIO, &GPIO_InitStructure);
	/* Configure CAN1 TX pin */
	GPIO_InitStructure.Pin = CANa_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(CANa_GPIO, &GPIO_InitStructure);

	/* Configure CAN2 RX pin */
	GPIO_InitStructure.Pin = CANb_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitPeripheral(CANb_GPIO, &GPIO_InitStructure);
	/* Configure CAN2 TX pin */
	GPIO_InitStructure.Pin = CANb_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitPeripheral(CANb_GPIO, &GPIO_InitStructure);
	/* Remap CAN1 GPIOs */
	GPIO_ConfigPinRemap(GPIO_RMP2_CAN1, ENABLE);
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
	CAN_Init(CAN1, &CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);
	CAN1_Filter_Init();
	CAN2_Filter_Init();
}

/**********************************************************
* 函数功能： stm32底层的can通信协议初始化。
* 参数：     无
* 说明：     无
**********************************************************/
void Can_Driver_Init(uint16_t baud)
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
 * 函数功能： RPDO0事件触发。
 * 参数：     ID代表节点地址。
 * 说明：     RPDO0映射0x6040（控制字）；
 *            RPDO0-COB-ID:0x200 + ID
 **********************************************************/
uint8_t ZLAC8015_PDO_Config(uint16_t id)
{
	int i,k;
	uint16_t s_id;
	struct SdoFrame* new_sdo_frame;
	uint8_t Init_sdo[19][8] = // 主站(单片机)，恢复从站的初始值 
	{// 写参数映射		
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x80},  // 失能Rpdo： 发送sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    //②Hex1400_02，传输形式：0xFE：事件触发；0xFF：定时器触发
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // 清除映射: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 控制字
		{0x23,0x00,0x16,0x02,0x20,0x00,0xFF,0x60},  // 60FF 目标速度
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //映射对象子索引个数	⑥Hex1600_0，启用【映射参数】 【实际映射多少组】 
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		// 读参数映射
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x80},  // 失能Tpdo： 发送sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,0x28,0x00,0x00,0x00},    //定时器触发时间40ms(刷新率为25Hz)
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // 消去个数: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 状态字
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},  // 606C 实时反馈速度
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//设置速度模式
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //②Hex1800_02，传输形式：0xFE：事件触发；0xFF：定时器触发 1个有效数据，设置TPDO1的传输类型，SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //⑥Hex1A00_0，启用【映射参数】 【实际映射多少组】 
		// 中菱伺服
		{0x23,0x08,0x20,0x00,0x00,0x00,0x00,0x00},  // 2008 起始速度 设置起始速度为0
		{0x23,0x83,0x60,0x00,0x64,0x00,0x00,0x00},  // 6083 设置电机加速时间100ms
		{0x23,0x84,0x60,0x00,0x64,0x00,0x00,0x00},  // 6084 设置电机减速时间100ms
	};
	s_id = 0x600 + id;	
	for (i = 0; i < 19; i++)
	{
		new_sdo_frame = SdoFramCereat();
		if (new_sdo_frame != NULL)
		{
			new_sdo_frame->ID = s_id;
			new_sdo_frame->mode = 0;
			for (k = 0;k < 8;k++)new_sdo_frame->data[k] = Init_sdo[i][k];			
		}
	}
	return 0x00;
}
uint8_t WANZER_PDO_Config(uint16_t id)// 万泽伺服
{
	int i, k;
	uint16_t s_id;	
	struct SdoFrame* new_sdo_frame;
	uint8_t Init_sdo[20][8] = // 主站(单片机)，恢复从站的初始值 
	{// 写参数映射		
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x80},  // 失能Rpdo： 发送sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    //②Hex1400_02，传输形式：0xFE：事件触发；0xFF：定时器触发
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // 清除映射: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 控制字
		{0x23,0x00,0x16,0x02,0x20,0x00,0xFF,0x60},  // 60FF 目标速度
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //映射对象子索引个数	⑥Hex1600_0，启用【映射参数】 【实际映射多少组】 
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		// 读参数映射
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x80},  // 失能Tpdo： 发送sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,0x05,0x00,0x00,0x00},    //定时器触发时间 4*0.5ms
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // 消去个数: 发送sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 状态字
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},  // 606C 实时反馈速度
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x00},  // 使能pdo:  发送sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//设置速度模式
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //②Hex1800_02，传输形式：0xFE：事件触发；0xFF：定时器触发 1个有效数据，设置TPDO1的传输类型，SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //⑥Hex1A00_0，启用【映射参数】 【实际映射多少组】 
		// 万泽伺服 扫描 601  40 00 20 01 00 00 00 00 
		{0x23,0x83,0x60,0x00,0x10,0x27,0x00,0x00},  // 6083 写入参数轮廓位置模式加速度 10000
		{0x23,0x84,0x60,0x00,0x10,0x27,0x00,0x00},  // 6084 写入参数轮廓位置模式减速度 10000
		{0x23,0xa4,0x60,0x01,0xA0,0x86,0x01,0x00},  // 60a4 01 写入参数轮廓加加速度 100000
		{0x23,0xa4,0x60,0x02,0xA0,0x86,0x01,0x00},  // 60a4 02 写入参数轮廓加加速度 100000
	};
	s_id = 0x600 + id;
	for (i = 0; i < 20; i++)
	{
		new_sdo_frame = SdoFramCereat();
		if (new_sdo_frame != NULL)
		{
			new_sdo_frame->ID = s_id;
			new_sdo_frame->mode = 0;
			for (k = 0;k < 8;k++)new_sdo_frame->data[k] = Init_sdo[i][k];
			
		}
	}
	return 0x00;
}

/**********************************************************
* 函数功能： NMT节点状态设置，对节点使能、状态切换。
* 参数：     ID代表节点地址；data0为功能选择，data1为节点地址。
* 说明：     激活驱动PDO传输
**********************************************************/
uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1)
{
	struct SdoFrame* new_sdo_frame;
	new_sdo_frame = SdoFramCereat();
	if (new_sdo_frame != NULL)
	{
		new_sdo_frame->ID = ID;
		new_sdo_frame->mode = 1;
		new_sdo_frame->data[0] = Data0;
		new_sdo_frame->data[1] = Data1;
	}
	return 0;  //
}

uint8_t CAN_SDOSend(CAN_Module* CANx)
{
	if (sdo_head != NULL)
	{
		CanTxMessage pTxMessage;
		int nIndex = 0;
		int nReturn = -1;
		uint8_t Data_Size = sdo_head->mode == 0 ? 8 : 2;
		pTxMessage.DLC = Data_Size;
		pTxMessage.IDE = CAN_Standard_Id;
		pTxMessage.RTR = CAN_RTRQ_Data;
		pTxMessage.StdId = sdo_head->ID;
		for (nIndex = 0; nIndex < Data_Size; nIndex++)
		{
			pTxMessage.Data[nIndex] = sdo_head->data[nIndex];
		}
		nIndex = 0;
		do
		{
			nIndex++;
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //发送成功，返回0~2(邮箱号)，失败返回0x04
		} while (nReturn == 0x04);
		struct SdoFrame* next_sdo = sdo_head->next;
		rt_free(sdo_head);
		sdo_head = next_sdo;
		return 1;

	}
	return 0;
}

/**********************************************************
 * 函数功能： CANPDO帧发送函数。
 * 参数：     number代表节点数量。
 * 说明：     该函数必须在PDO配置完成之后运行。
 * 返回值：   发送成功数量
 **********************************************************/
uint8_t CAN_PDOSend(uint32_t number, CAN_Module* CANx)
{
	CanTxMessage pTxMessage;
	MOTOR_RPDO* rpdo;
	int target_velocity;
	int nReturn = 0x04;
	int nCount = 0;
	int sCount = 0;

	pTxMessage.IDE = CAN_Standard_Id;
	pTxMessage.RTR = CAN_RTRQ_Data;
	pTxMessage.DLC = 6;//< 

	do
	{
		rpdo = &mrd[send_number - 1];
		if (rpdo->d.online)
		{
			pTxMessage.StdId = 0x200 + rpdo->d.mapping;
			pTxMessage.Data[0] = rpdo->d.ctrl.cd & 0xff;
			pTxMessage.Data[1] = (rpdo->d.ctrl.cd >> 8) & 0xff;
			target_velocity = rpdo->d.target_velocity;
			pTxMessage.Data[2] = target_velocity & 0xFF;       // 低八位写入 Data[2]
			pTxMessage.Data[3] = (target_velocity >> 8) & 0xFF; // 
			pTxMessage.Data[4] = (target_velocity >> 16) & 0xFF; // 
			pTxMessage.Data[5] = (target_velocity >> 24) & 0xFF; // 高八位写入 Data[5]
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //发送成功，返回0
		}
		else
		{
			sCount++;
			if (sCount >= number)break;
			nReturn = 0;
		}
		if (nReturn != 0x04)
		{
			nCount++;
			send_number++;//< 发送下一个
			if (send_number > number)send_number = 1;
		}
		else break;
	} while (1);


	return nCount;
}

/**************************************************************************
函数功能：CAN PDO任务
入口参数：无
返回  值：无
**************************************************************************/
void Can_task(void* pvParameters)
{
	uint16_t* pdu = (uint16_t*)pvParameters;
	int buff_cnt = 0;//<
	while (1)
	{
		rt_thread_delay(20);   //< 1ms
		CAN_SDOSend(CAN1);
		if (buff_cnt > 0)
		{
			buff_cnt--;
		}
		CAN_PDOSend(Motor_Number, CAN1);

		if (pdu[receive_buff_num] != 0)
		{
			buff_cnt += pdu[receive_buff_num];
			//< 数据拷贝过来
			pdu[receive_buff_num] = 0;
		}

		pdu[empty_buff_num] = 30 - buff_cnt;
	}

}

void CanIRQProcess(CAN_Module* CANx)
{
	CanTxMessage RxMessage;
	CAN_ReceiveMessage(CANx, 0, &RxMessage);
	// 电机PDO反馈
	if (RxMessage.StdId >= 0x181 && RxMessage.StdId <= 0X1FF)
	{//< 电机PDO反馈
		int id = RxMessage.StdId - 0x180; //获取CAN ID 号
		int i = 0;
		MOTOR_TPDO* md;
		if (id < 10)
		{
			for (i = 0;i < MAX_MOTOR_NUMBER;i++)
			{
				if (id == mrd[i].d.mapping)
				{
					// 找到映射后的ID
					break;
				}
			}
			mrd[i].d.online = 1;
			md = &mtd[i];
			md->d.heartbeat = 0;
			i = 0;
			md->d.status.sd = RxMessage.Data[i++];
			md->d.status.sd |= RxMessage.Data[i++] << 8;
			md->d.current_velocity = RxMessage.Data[i++];
			md->d.current_velocity |= RxMessage.Data[i++] << 8;
			md->d.current_velocity |= RxMessage.Data[i++] << 16;
			md->d.current_velocity |= RxMessage.Data[i++] << 24;
		}
	}
	else
	{//< SDO 处理

	}
	CAN_ClearFlag(CANx, CAN_INT_FMP0);
}

void CAN2_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN2);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN1);
}



