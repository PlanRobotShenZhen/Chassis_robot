#ifndef __user_can__h
#define __user_can__h

#include "can.h"

#define CAN_TASK_PRIO		20     //�������ȼ�
#define CAN_STK_SIZE 		128   //�����ջ��С

#define LB_CAN_ID   0x01 //0x01 //0x04//< ��ǰ
#define LF_CAN_ID   0x02 //0x02 //0x05//< ���
#define RB_CAN_ID   0x03 //0x03 //0x07//< ��ǰ  
#define RF_CAN_ID   0x04 //0x04 //0x06//< �Һ�       

extern uint8_t can_id_map[4];
//�����೤ʱ��û������������Ϊ�ӻ�����
extern int g_nHeart_Time_LB;
extern int g_nHeart_Time_LF;
extern int g_nHeart_Time_RF;
extern int g_nHeart_Time_RB;

void Can_Driver_Init(uint16_t baud);
uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1);//NMT����
uint8_t ZLAC8015_PDO_Config(uint16_t id);// �����ŷ�
uint8_t WANZER_PDO_Config(uint16_t id);// �����ŷ�
uint8_t IsSdoEmpty(void);
void Can_task(void* pvParameters);

#endif

