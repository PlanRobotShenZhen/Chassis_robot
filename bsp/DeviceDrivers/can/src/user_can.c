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


MOTOR_TPDO mtd[MAX_MOTOR_NUMBER];//< ����pdo
MOTOR_RPDO mrd[MAX_MOTOR_NUMBER];//< ����pdo

//����ʱ��û����������1000
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
	RCC_EnableAPB2PeriphClk(CANb_CLK, ENABLE);//CANa��CANbͬʱ��
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
* �������ܣ� stm32�ײ��canͨ��Э���ʼ����
* ������     ��
* ˵����     ��
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
		nReturn = CAN_TransmitMessage(CAN1, &pTxMessage); //���ͳɹ�������0~2(�����)��ʧ�ܷ���0x04
		errCount++;
		if (errCount > 10)
		{
			ret = 1;
			break;
		}
	} while (nReturn == 0x04);
	return ret;
}


/*�������������EEPROM��δ����*/
uint8_t Save_EEPROM(uint8_t ID)
{
	uint8_t Data[8] = {0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	return DevCANOpen_send_sdo(0x600 + ID, 0x08, Data);
}


/*�ָ��������ã�δ����*/
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
			sdo_head = new_sdo_frame;//����һ��ͷ���
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
 * �������ܣ� ZLAC_PDO�¼����á�
 * ������     ID����ڵ��ַ��
 * ˵����     ����PDO��
 **********************************************************/
void ZLAC8015_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)// �����ŷ�
{
	uint8_t v[7];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//��ʱ��50ms
	v[3] = 0xFF;		//�ٶ�ģʽ��ַ
	v[4] = 0x20;		//4�ֽ�		
	v[5] = 0x29;		//ӳ���ַ
	v[6] = 0x20;		//ӳ���ַ

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//λ��ģʽ��ַ
	}else if(mode == torque_mode){
		v[3] = 0x71;	//ת��ģʽ��ַ
		v[4] = 0x10;	//2�ֽ�
		v[5] = 0x77;
		v[6] = 0x60;
	}

	uint8_t Init_sdo[][8] = {// ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ
		// ������ӳ�䣨TPDO0��
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  	//ʧ��Tpdo0
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    	//��ʱ������ʱ�� 20/50ms(ˢ����Ϊ50/20Hz)	
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  	//���ӳ��
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  	//6041  ״̬��
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x64,0x60},  	//6064  ��ǰλ��
		{0x23,0x00,0x1A,0x03,0x10,0x00,0x26,0x20},  	//2026  PCB�¶�
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00}, 		//����PDO��COB-ID
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    	//��ʱ������
		{0x2F,0x00,0x1A,0x00,0x03,0x00,0x00,0x00},    	//ӳ���������������
		// ������ӳ�䣨TPDO1��
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80},  	//ʧ��Tpdo1
		{0x2B,0x01,0x18,0x05,0x32,0x00,0x00,0x00},    	//��ʱ������ʱ�� 50ms(ˢ����Ϊ20Hz)
		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  	//���ӳ��
		{0x23,0x01,0x1A,0x01,0x20,0x00,0x6C,0x60},  	//606C  �ٶȷ���	
		{0x23,0x01,0x1A,0x02,0x10,0x00,v[5],v[6]},  	//2029/6077 ĸ�ߵ�ѹ/ת�ط���				
		{0x23,0x02,0x1A,0x01,0x10,0x00,0x3F,0x60},  	//603F  ��ȡ������					
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, 		//����PDO��COB-ID
		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    	//��ʱ������
		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    	//ӳ���������������				
		// д����ӳ�䣨RPDO0��
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  	//ʧ��Rpdo0
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    	//��Hex1400_02��������ʽ��0xFE���¼�������0xFF����ʱ������
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  	//���ӳ��
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  	//6040 ������
    	{0x23,0x00,0x16,0x02,v[4],0x00,v[3],0x60},		//�˶�ģʽ
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},   	//ӳ���������������
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  	//����PDO��COB-ID	
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
	if(mode == position_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00},  //����λ��ģʽ	
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//���ü���ʱ��50ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//���ü���ʱ��50ms
			{0x23,0x81,0x60,0x00,0xFC,0x08,0x00,0x00},  //��������ٶ�2300r/min	
			{0x2B,0x5D,0x60,0x00,0x03,0x00,0x00,0x00},  //HALT��ͣ	
			{0x2B,0x17,0x20,0x00,0x70,0x17,0x00,0x00},  //�������ֵ6000*10counts
		};																																//���PDO����															//��ת��ƽ	
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}else if(mode == speed_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},  //�����ٶ�ģʽ	
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//���ü���ʱ��100ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//���ü���ʱ��100ms
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));	
		Driver_JT_Inv(id);			//��ת��ƽ	
	}else if(mode == torque_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x04,0x00,0x00,0x00},  //��������ģʽ
			{0x23,0x83,0x60,0x00,al,ah,0x00,0x00},  	//���ü���ʱ��100ms
			{0x23,0x84,0x60,0x00,dl,dh,0x00,0x00},  	//���ü���ʱ��100ms
			{0x23,0x87,0x60,0x00,0x10,0x27,0x00,0x00},  //ת��б��10000	
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}
}

/**********************************************************
 * �������ܣ� ����������ӵ�ƽ��ת��
 * ������     �ޡ�
 * ˵����     ��Դ��Ĭ����Ƶ͵�ƽΪ��Ч��ƽ��������ͣ�Ǹߵ�ƽ��Ч��Ϊ�������Դ�壬��Ҫ��ת��ƽ��
 **********************************************************/
void Driver_JT_Inv(uint8_t id)
{
	uint8_t Init_sdo[][8] = {0x2B,0x30,0x20,0x01,0x01,0x00,0x00,0x00}; // ��ת�������X0��X1�ĵ�ƽ������Bit0��1��1
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
}


/**********************************************************
 * �������ܣ� RPDO0�¼�������
 * ������     ID����ڵ��ַ��
 * ˵����     RPDO0ӳ��0x6040�������֣���
 *            RPDO0-COB-ID:0x200 + ID
 **********************************************************/
void ZLAC8015D_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)
{
	uint8_t Init_sdo[][8] = {// ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ 	
		{0x2B,0x0F,0x20,0x00,0x01,0x00,0x00,0x00},	//����ͬ������
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//�����ٶ�ģʽ
		//// �����ŷ�
		{0x23,0x83,0x60,0x01,al,ah,0x00,0x00},  	// 6083 01�������ü���ʱ��100ms
		{0x23,0x83,0x60,0x02,al,ah,0x00,0x00},  	// 6083 02�ҵ�����ü���ʱ��100ms
		{0x23,0x84,0x60,0x01,dl,dh,0x00,0x00},  	// 6084 01�������ü���ʱ��100ms
		{0x23,0x84,0x60,0x02,dl,dh,0x00,0x00},  	// 6084 02�ҵ�����ü���ʱ��100ms
		// ������ӳ�䣨TPDO0��
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // ��ȥ����: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 ״̬��
		{0x23,0x00,0x1A,0x02,0x20,0x03,0x6C,0x60},  // 606C ʵʱ�����ٶ�
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},  //��Hex1800_02��������ʽ��0xFE���¼�������0xFF����ʱ������ 1����Ч���ݣ�����TPDO1�Ĵ������ͣ�SYNC 
		{0x2B,0x00,0x18,0x05,0x32,0x00,0x00,0x00},  //��ʱ������ʱ��50ms(ˢ����Ϊ20Hz)
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},  //��Hex1A00_0�����á�ӳ������� ��ʵ��ӳ������顿 
		// д����ӳ�䣨RPDO0��
		{0x2F,0x01,0x16,0x00,0x00,0x00,0x00,0x00},  // ���ӳ��: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x01,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 ������
		{0x23,0x01,0x16,0x02,0x20,0x03,0xFF,0x60},  // 60FF Ŀ���ٶ�
		{0x2F,0x01,0x16,0x00,0x02,0x00,0x00,0x00},  //ӳ���������������	��Hex1600_0�����á�ӳ������� ��ʵ��ӳ������顿 
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
}

/**********************************************************
 * �������ܣ� WZ_PDO�¼����á�
 * ������     ID����ڵ��ַ��
 * ˵����     ����PDO��
 **********************************************************/
void WANZER_PDO_Config(uint8_t id, uint8_t mode)// �����ŷ�
{
	uint8_t v[4];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//��ʱ��50ms
	v[3] = 0xFF;		//�ٶ�ģʽ��ַ

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//λ��ģʽ��ַ
	}
	uint8_t Init_sdo[][8]={ // ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ 
		// ������ӳ�䣨TPDO0��
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  		//ʧ��Tpdo0
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    		//��ʱ������ʱ�� 20/50ms(ˢ����Ϊ50/20Hz)	
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  			//���ӳ��
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  			//6041  ״̬��
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x64,0x60},  			//6064  ��ǰλ��
		{0x23,0x00,0x1A,0x03,0x10,0x04,0x04,0x20},  			//2004  PCB�¶�	
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00}, 		//����PDO��COB-ID
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    		//��ʱ������
		{0x2F,0x00,0x1A,0x00,0x03,0x00,0x00,0x00},    		//ӳ���������������
		// ������ӳ�䣨TPDO1��
//		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80},  		//ʧ��Tpdo1
//		{0x2B,0x01,0x18,0x05,v[2],0x00,0x00,0x00},    		//��ʱ������ʱ�� 50ms(ˢ����Ϊ20Hz)
//		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  			//���ӳ��
//		{0x23,0x01,0x1A,0x01,0x20,0x00,0x6C,0x60},  			//606C  �ٶȷ���
//		{0x23,0x01,0x1A,0x02,0x10,0x02,0x04,0x20},  			//2004  ĸ�ߵ�ѹ
//		{0x23,0x01,0x1A,0x03,0x10,0x03,0x04,0x20},  			//2004  ĸ�ߵ���	
//		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, 		//����PDO��COB-ID
//		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    		//��ʱ������
//		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    		//ӳ���������������	
		// д����ӳ�䣨RPDO0��
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  				//ʧ��Rpdo0
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    		//��Hex1400_02��������ʽ��0xFE���¼�������0xFF����ʱ������
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  			//���ӳ��
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  			//6040 ������
    	{0x23,0x00,0x16,0x02,0x20,0x00,v[3],0x60},  			//60FF/7A �ٶȡ�λ��ģʽ
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},   		  //ӳ���������������
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  				// ����PDO��COB-ID	
	};
	Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));
	if(mode == position_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00},  //����λ��ģʽ	
			{0x23,0x65,0x60,0x00,0x40,0x0D,0x03,0x00},  //�������ֵ20*10000counts
			{0x23,0x66,0x60,0x00,0x88,0x13,0x00,0x00},  //�������ֵ 5000ms
			{0x23,0x81,0x60,0x00,0xC0,0x27,0x09,0x00},  // 6081 д���������λ��ģʽ���滮�ٶ� 600000			
			{0x23,0x83,0x60,0x00,0xD0,0x09,0x00,0x00},  // 6083 д���������λ��ģʽ���ٶ� 2512
			{0x23,0x84,0x60,0x00,0xD0,0x09,0x00,0x00},  // 6084 д���������λ��ģʽ���ٶ� 2512
			{0x23,0xa4,0x60,0x01,0xD0,0x09,0x00,0x00},  // 60a4 01 д����������Ӽ��ٶ� 2512
			{0x23,0xa4,0x60,0x02,0xD0,0x09,0x00,0x00},  // 60a4 02 д����������Ӽ��ٶ� 2512	
		};																																//���PDO����															//��ת��ƽ	
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));
	}else if(mode == speed_mode){
		uint8_t init_sdo[][8]={
			{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},  //�����ٶ�ģʽ	
			{0x23,0x83,0x60,0x00,0x10,0x27,0x00,0x00},  // 6083 д���������λ��ģʽ���ٶ� 10000
			{0x23,0x84,0x60,0x00,0x10,0x27,0x00,0x00},  // 6084 д���������λ��ģʽ���ٶ� 10000
			{0x23,0xa4,0x60,0x01,0xA0,0x86,0x01,0x00},  // 60a4 01 д����������Ӽ��ٶ� 100000
			{0x23,0xa4,0x60,0x02,0xA0,0x86,0x01,0x00},  // 60a4 02 д����������Ӽ��ٶ� 100000
		};
		Add_Sdo_Linked_List(id, init_sdo, sizeof(init_sdo)/sizeof(init_sdo[0]));	
	}
}

/**********************************************************
 * �������ܣ� PL_PDO�¼����á�
 * ������     ID����ڵ��ַ��
 * ˵����     ����PDO��
 **********************************************************/
void PLAN_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl)// �����ŷ�
{
	uint8_t v[5];
	v[0] = 0x80 + id;	//tpdo_id
	v[1] = id;			//rpdo_id
	v[2] = 0x32;		//��ʱ��50ms
	v[3] = 0xFF;		//�ٶ�ģʽ��ַ
	v[4] = 0x20;		//4�ֽ�		

	if(mode == position_mode){
		v[2] = 0x14;		//20ms
		v[3] = 0x7A;		//λ��ģʽ��ַ
	}else if(mode == torque_mode){
		v[3] = 0x71;	//ת��ģʽ��ַ
		v[4] = 0x10;	//2�ֽ�
	}
	uint8_t Init_sdo[][8]={ // ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ 	
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x80},  // ʧ��Rpdo�� ����sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},  //��Hex1400_02��������ʽ��0xFE���¼�������0xFF����ʱ������
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // ���ӳ��: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 ������
		{0x23,0x00,0x16,0x02,v[4],0x00,v[3],0x60},  // 60FF Ŀ���ٶ�
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //ӳ���������������	��Hex1600_0�����á�ӳ������� ��ʵ��ӳ������顿 
		{0x23,0x00,0x14,0x01,v[1],0x02,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		// ������ӳ��TPDO0
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x80},  // ʧ��Tpdo�� ����sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,v[2],0x00,0x00,0x00},    //��ʱ������ʱ�� 50ms
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // ��ȥ����: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 ״̬��
		{0x23,0x00,0x1A,0x02,v[4],0x00,0x6C,0x60},  // 606C ʵʱ�����ٶ�
		{0x23,0x00,0x18,0x01,v[0],0x01,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//�����ٶ�ģʽ
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //��Hex1800_02��������ʽ��0xFE���¼�������0xFF����ʱ������ 1����Ч���ݣ�����TPDO1�Ĵ������ͣ�SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //��Hex1A00_0�����á�ӳ������� ��ʵ��ӳ������顿
		// ������ӳ�䣨TPDO1��
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x80}, //ʧ��Tpdo1
		{0x2B,0x01,0x18,0x05,v[2],0x00,0x00,0x00},    	//��ʱ������ʱ�� 100ms(ˢ����Ϊ10Hz)
		{0x2F,0x01,0x1A,0x00,0x00,0x00,0x00,0x00},  	//���ӳ��
		{0x23,0x01,0x1A,0x01,0x10,0x0d,0x00,0x20},  	//2000 0d RPM		
		{0x23,0x01,0x1A,0x02,0x10,0x00,0x78,0x60},  	//6078 00 ��������	
		{0x23,0x01,0x1A,0x03,v[4],0x00,0x64,0x60},  	//606C 00 λ�÷���	
		{0x23,0x01,0x18,0x01,v[0],0x02,0x00,0x00}, //����PDO��COB-ID
		{0x2F,0x01,0x18,0x02,0xFF,0x00,0x00,0x00},    //��Hex1801_02��������ʽ��0xFE���¼�������0xFF����ʱ������ 1����Ч���ݣ�����TPDO1�Ĵ������ͣ�SYNC 
		{0x2F,0x01,0x1A,0x00,0x03,0x00,0x00,0x00},    //��Hex1A01_0�����á�ӳ������� ��ʵ��ӳ������顿 
		// ������ӳ�䣨TPDO2��
		{0x23,0x02,0x18,0x01,v[0],0x03,0x00,0x80}, //ʧ��Tpdo2
		{0x2B,0x02,0x18,0x05,v[2],0x00,0x00,0x00},    	//��ʱ������ʱ�� 100ms(ˢ����Ϊ10Hz)
		{0x2F,0x02,0x1A,0x00,0x00,0x00,0x00,0x00},  	//���ӳ��
		{0x23,0x02,0x1A,0x01,0x10,0x00,0x3f,0x60},  	//603f 00 Errorcode		
		{0x23,0x02,0x1A,0x02,0x08,0x00,0x61,0x60},  	//6061 00 ģʽ
		{0x23,0x02,0x18,0x01,v[0],0x03,0x00,0x00}, //����PDO��COB-ID
		{0x2F,0x02,0x18,0x02,0xFF,0x00,0x00,0x00},    
		{0x2F,0x02,0x1A,0x00,0x02,0x00,0x00,0x00},    
	};
		Add_Sdo_Linked_List(id, Init_sdo, sizeof(Init_sdo)/sizeof(Init_sdo[0]));	
}

/**********************************************************
* �������ܣ� NMT�ڵ�״̬���ã��Խڵ�ʹ�ܡ�״̬�л���
* ������     ID����ڵ��ַ��data0Ϊ����ѡ��data1Ϊ�ڵ��ַ��
* ˵����     ��������PDO����
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
			point_data_head = new_point_frame;//����һ��ͷ���
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
�������ܣ�CAN PDO����
��ڲ�������
����  ֵ����
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
�������ܣ����CAN֮RPDO��������
��ڲ������� 
����  ֵ����
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
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //���ͳɹ�������0~2(�����)��ʧ�ܷ���0x04
		} while (nReturn == 0x04);
		struct SdoFrame* next_sdo = sdo_head->next;
		rt_free(sdo_head);
		sdo_head = next_sdo;
	}
}

/**********************************************************
 * �������ܣ� CANPDO֡���ͺ�����
 * ������     number����ڵ�������
 * ˵����     �ú���������PDO�������֮�����С�
 * ����ֵ��   ���ͳɹ�����
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
			pTxMessage.Data[0] = rpdo->d.ctrl.cw & 0xFF;      // д�������
			pTxMessage.Data[1] = (rpdo->d.ctrl.cw >> 8) & 0xFF;
			switch (x_type){
				case servo_zlac:
					pTxMessage.Data[2] = rpdo->d.target_pos_vel & 0xFF;       // �Ͱ�λд�� Data[2]
					pTxMessage.Data[3] = (rpdo->d.target_pos_vel >> 8) & 0xFF; // 
					pTxMessage.Data[4] = (rpdo->d.target_pos_vel >> 16) & 0xFF; // 
					pTxMessage.Data[5] = (rpdo->d.target_pos_vel >> 24) & 0xFF; // �߰�λд�� Data[5]
					break;
				case servo_wanze:
					target_pos_pulse = rpdo->d.target_pos_vel;
					switch (x_sport_mode){
						case speed_mode:
							target_pos_pulse = rpdo->d.target_pos_vel * ENCODER_LINES / MINTOSEC; //Ĭ�ϵ�λ��ͬ����Ҫ����λ��rpmת����pulse
							break;
						case position_mode://Ĭ�ϵ�λ��ͬ
							break;
					}
					pTxMessage.Data[2] = target_pos_pulse & 0xFF;       // �Ͱ�λд�� Data[2]
					pTxMessage.Data[3] = (target_pos_pulse >> 8) & 0xFF; // 
					pTxMessage.Data[4] = (target_pos_pulse >> 16) & 0xFF; // 
					pTxMessage.Data[5] = (target_pos_pulse >> 24) & 0xFF; // �߰�λд�� Data[5]
					break;
				case servo_zlacd:
					pTxMessage.Data[2] = mrd[0].d.target_pos_vel & 0xFF;       	// ����Ŀ���ٶȣ��Ͱ�λд�� Data[2]
					pTxMessage.Data[3] = (mrd[0].d.target_pos_vel >> 8) & 0xFF; // 
					pTxMessage.Data[4] = mrd[1].d.target_pos_vel & 0xFF; 		// �ҵ��Ŀ���ٶȣ��Ͱ�λд�� Data[2]
					pTxMessage.Data[5] = (mrd[1].d.target_pos_vel >> 8) & 0xFF; // �߰�λд�� Data[5]
					break;
				case servo_plan:
					/* code */
					break;
			}
			nReturn = CAN_TransmitMessage(CANx, &pTxMessage); //������ʧ�ܣ�����0x04
		if (nReturn != 0x04){
			send_number++;//< ������һ��
			if (send_number >= number){
				send_number = 0; 
			}
		}else{			
		break;
		}
	} while (1);
}

/**********************************************************
 * �������ܣ� ����Ϣ���뵽sdo����
 * ������     ��
 * ����ֵ��   ��
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
 * �������ܣ� ����һ�� SDO ֡
 * ������     ��
 * ˵����     �ú������ڶ�̬����һ�� SDO ֡�ڵ㣬��������ӵ� SDO �����С�
 * ����ֵ��   ����ɹ���������ӵ���������ָ���´����� SDO ֡�ڵ��ָ�룬���򷵻� NULL��
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
			sdo_head = new_sdo_frame;//����һ��ͷ���
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
 * �������ܣ� �ж� SDO �����Ƿ�Ϊ��
 * ������     ��
 * ˵����     �ú������ڼ�� SDO �����Ƿ�Ϊ�գ�ͨ�����ͷָ���Ƿ�Ϊ�����жϡ�
 * ����ֵ��   1 ��ʾ SDO ����Ϊ�գ�0 ��ʾ SDO ���зǿ�
 **********************************************************/
uint8_t IsSdoEmpty(void)
{
	return sdo_head == NULL;
}
