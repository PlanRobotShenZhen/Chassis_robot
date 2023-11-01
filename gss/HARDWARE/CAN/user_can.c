#include "user_can.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "usartx.h"
#include "motor_data.h"
#include <stdlib.h>

struct SdoFrame
{
	uint16_t ID;
	uint8_t mode;//<0:sdo,1:nmt
	uint8_t data[8];
	struct SdoFrame* next;
};
static struct SdoFrame* sdo_head = NULL;

//����ʱ��û����������1000
int g_nHeart_Time_LB = 0;
int g_nHeart_Time_LF = 0;
int g_nHeart_Time_RF = 0;
int g_nHeart_Time_RB = 0;
int send_number = 1;

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
	struct SdoFrame* new_sdo_frame;
	struct SdoFrame* next_sdo_frame;

	new_sdo_frame = (struct SdoFrame*)malloc(sizeof(struct SdoFrame));
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
 * �������ܣ� RPDO0�¼�������
 * ������     ID����ڵ��ַ��
 * ˵����     RPDO0ӳ��0x6040�������֣���
 *            RPDO0-COB-ID:0x200 + ID
 **********************************************************/
uint8_t ZLAC8015_PDO_Config(uint16_t id)
{
	int i,k;
	uint8_t Init_sdo[19][8] = // ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ 
	{// д����ӳ��		
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x80},  // ʧ��Rpdo�� ����sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    //��Hex1400_02��������ʽ��0xFE���¼�������0xFF����ʱ������
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // ���ӳ��: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 ������
		{0x23,0x00,0x16,0x02,0x20,0x00,0xFF,0x60},  // 60FF Ŀ���ٶ�
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //ӳ���������������	��Hex1600_0�����á�ӳ������� ��ʵ��ӳ������顿 
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		// ������ӳ��
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x80},  // ʧ��Tpdo�� ����sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,0x05,0x00,0x00,0x00},    //��ʱ������ʱ�� 4*0.5ms
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // ��ȥ����: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 ״̬��
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},  // 606C ʵʱ�����ٶ�
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//�����ٶ�ģʽ
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //��Hex1800_02��������ʽ��0xFE���¼�������0xFF����ʱ������ 1����Ч���ݣ�����TPDO1�Ĵ������ͣ�SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //��Hex1A00_0�����á�ӳ������� ��ʵ��ӳ������顿 
		// �����ŷ�
		{0x23,0x08,0x20,0x00,0x00,0x00,0x00,0x00},  // 2008 ��ʼ�ٶ� ������ʼ�ٶ�Ϊ0
		{0x23,0x83,0x60,0x00,0x64,0x00,0x00,0x00},  // 6083 ���õ������ʱ��100ms
		{0x23,0x84,0x60,0x00,0x64,0x00,0x00,0x00},  // 6084 ���õ������ʱ��100ms
	};
	struct SdoFrame* new_sdo_frame;
	uint16_t s_id = 0x600 + id;	
	for (i = 0; i < 19; )
	{
		new_sdo_frame = SdoFramCereat();
		if (new_sdo_frame != NULL)
		{
			new_sdo_frame->ID = s_id;
			new_sdo_frame->mode = 0;
			//memcpy(new_sdo_frame->data, &Init_sdo[i], 8);
			for (k = 0;k < 8;k++)new_sdo_frame->data[k] = Init_sdo[i][k];
			i++;
		}
	}
	return 0x00;
}
uint8_t WANZER_PDO_Config(uint16_t id)// �����ŷ�
{
	int i, k;
	uint8_t Init_sdo[20][8] = // ��վ(��Ƭ��)���ָ���վ�ĳ�ʼֵ 
	{// д����ӳ��		
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x80},  // ʧ��Rpdo�� ����sdo  600+id,23 00 14 01 01 02 00 80
		{0x2F,0x00,0x14,0x02,0xFE,0x00,0x00,0x00},    //��Hex1400_02��������ʽ��0xFE���¼�������0xFF����ʱ������
		{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},  // ���ӳ��: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},  // 6040 ������
		{0x23,0x00,0x16,0x02,0x20,0x00,0xFF,0x60},  // 60FF Ŀ���ٶ�
		{0x2F,0x00,0x16,0x00,0x02,0x00,0x00,0x00},    //ӳ���������������	��Hex1600_0�����á�ӳ������� ��ʵ��ӳ������顿 
		{0x23,0x00,0x14,0x01,id,0x02,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		// ������ӳ��
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x80},  // ʧ��Tpdo�� ����sdo  600+id,22 00 14 01 01 02 00 80
		{0x2B,0x00,0x18,0x05,0x05,0x00,0x00,0x00},    //��ʱ������ʱ�� 4*0.5ms
		{0x2F,0x00,0x1A,0x00,0x00,0x00,0x00,0x00},  // ��ȥ����: ����sdo 600+id,22 00 16 00 00 00 00 00
		{0x23,0x00,0x1A,0x01,0x10,0x00,0x41,0x60},  // 6041 ״̬��
		{0x23,0x00,0x1A,0x02,0x20,0x00,0x6C,0x60},  // 606C ʵʱ�����ٶ�
		{0x23,0x00,0x18,0x01,0x80 + id,0x01,0x00,0x00},  // ʹ��pdo:  ����sdo 600+id,22 00 14 01 01 02 00 00
		{0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00},	//�����ٶ�ģʽ
		{0x2F,0x00,0x18,0x02,0xFF,0x00,0x00,0x00},    //��Hex1800_02��������ʽ��0xFE���¼�������0xFF����ʱ������ 1����Ч���ݣ�����TPDO1�Ĵ������ͣ�SYNC 
		{0x2F,0x00,0x1A,0x00,0x02,0x00,0x00,0x00},    //��Hex1A00_0�����á�ӳ������� ��ʵ��ӳ������顿 
		// �����ŷ� ɨ�� 601  40 00 20 01 00 00 00 00 
		{0x23,0x83,0x60,0x00,0x10,0x27,0x00,0x00},  // 6083 д���������λ��ģʽ���ٶ� 10000
		{0x23,0x84,0x60,0x00,0x10,0x27,0x00,0x00},  // 6084 д���������λ��ģʽ���ٶ� 10000
		{0x23,0xa4,0x60,0x01,0xA0,0x86,0x01,0x00},  // 60a4 01 д����������Ӽ��ٶ� 100000
		{0x23,0xa4,0x60,0x02,0xA0,0x86,0x01,0x00},  // 60a4 02 д����������Ӽ��ٶ� 100000
	};
	struct SdoFrame* new_sdo_frame;
	uint16_t s_id = 0x600 + id;
	for (i = 0; i < 20; )
	{
		new_sdo_frame = SdoFramCereat();
		if (new_sdo_frame != NULL)
		{
			new_sdo_frame->ID = s_id;
			new_sdo_frame->mode = 0;
			//memcpy(new_sdo_frame->data, &Init_sdo[i], 8);
			for (k = 0;k < 8;k++)new_sdo_frame->data[k] = Init_sdo[i][k];
			i++;
		}
	}
	return 0x00;
}

/**********************************************************
* �������ܣ� NMT�ڵ�״̬���ã��Խڵ�ʹ�ܡ�״̬�л���
* ������     ID����ڵ��ַ��data0Ϊ����ѡ��data1Ϊ�ڵ��ַ��
* ˵����     ��������PDO����
**********************************************************/
uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1)
{
	struct SdoFrame* new_sdo_frame;
	new_sdo_frame = SdoFramCereat();
	new_sdo_frame->ID = ID;
	new_sdo_frame->mode = 1;
	new_sdo_frame->data[0] = Data0;
	new_sdo_frame->data[1] = Data1;
	return 0;  //
}

uint8_t CAN_SDOSend(void)
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
		do
		{
			nReturn = CAN_TransmitMessage(CAN1, &pTxMessage); //���ͳɹ�������0~2(�����)��ʧ�ܷ���0x04
		} while (nReturn == 0x04);
		struct SdoFrame* next_sdo = sdo_head->next;
		free(sdo_head);
		sdo_head = next_sdo;
		return 1;

	}
	return 0;
}

/**********************************************************
 * �������ܣ� CANPDO֡���ͺ�����
 * ������     number����ڵ�������
 * ˵����     �ú���������PDO�������֮�����С�
 * ����ֵ��   ���ͳɹ�����
 **********************************************************/
uint8_t CAN_PDOSend(uint32_t number)
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
			pTxMessage.Data[2] = target_velocity & 0xFF;       // �Ͱ�λд�� Data[2]
			pTxMessage.Data[3] = (target_velocity >> 8) & 0xFF; // 
			pTxMessage.Data[4] = (target_velocity >> 16) & 0xFF; // 
			pTxMessage.Data[5] = (target_velocity >> 24) & 0xFF; // �߰�λд�� Data[5]
			nReturn = CAN_TransmitMessage(CAN1, &pTxMessage); //���ͳɹ�������0
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
			send_number++;//< ������һ��
			if (send_number > number)send_number = 1;
		}
		else break;
	} while (1);


	return nCount;
}

/**************************************************************************
�������ܣ�CAN PDO����
��ڲ�������
����  ֵ����
**************************************************************************/
void Can_task(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt(); //��ȡ��ϵͳ�ϴ�ִ��ʱ��
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//��������1000Hz��Ƶ������
		CAN_SDOSend();
		CAN_PDOSend(Motor_Number);
	}

}


