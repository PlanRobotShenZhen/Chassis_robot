/****************************************
Usart2�ǽ��պ�ģң�������͹��������ݣ�
Usart3������λ��ROSͨ�ŵġ�
****************************************/

#ifndef __USARTX_H
#define __USARTX_H 

#include "stdio.h"
#include "n32g45x_conf.h"
#include "bsp.h"




#define USART1_RX_MAXBUFF   2560 		// ����������󻺳���
#define USART1_TX_MAXBUFF   256 		// ����������󻺳���
#define USART4_RX_MAXBUFF 500
#define USART4_TX_MAXBUFF 25
/*��-------------��ģģ��--------------*/
#define Max_BUFF_Len      200          //һ֡���ݵĳ��ȣ�25���ֽ�
#define VEL_BASE_VALUE   1023         //ch3ͨ����׼ֵ���ٶ�
#define DIR_BASE_VALUE   1023         //ch1ͨ����׼ֵ������
#define LIMIT_MAX_VAL    1807         //�޷�ֵ��max
#define LIMIT_MIN_VAL    240          //�޷�ֵ��min
#define MOTOR_MULTI       3           //�ٶȱ���

#define SPEED_LEVEL1        240       //���������ٶȵ�λ�Ļ�׼ֵ,swb���ƣ�ch6ͨ��
#define SPEED_LEVEL2        1023
#define SPEED_LEVEL3        1807

#define SPEED_LOW              500        //���������ٶȵȼ�
#define SPEED_MIDDLE           1000
#define SPEED_HIGH            1300
#define SPEED_DIR_LOW          300    //��������ת���ٶȵȼ�
#define SPEED_DIR_MIDDLE       500
#define SPEED_DIR_HIGH        800

#define TURN_OFF_REMOTE      240      //���庽ģ���أ�ch7ͨ��
#define TURN_ON_REMOTE       1807  

#define LIGHT_BASE           1017     //���峵�Ƶ�������׼ֵ
#define LIGHT_MAX            1807
#define LIGHT_MIN            240




extern int g_nLastVelocity;           //��ŵ�ǰ���ٶȺ���һ�̵��ٶ�
extern int g_nCurrentVelocity;

/*��-------------����λ��ͨ��Э����غ궨��--------------*/
#define FRAME_HEADER 0X7B             //�������ݵ�֡ͷ,"{"
#define FRAME_TAIL 0X7D               //�������ݵ�֡β,"}"
#define SEND_DATA_SIZE 34
#define RECEIVE_DATA_SIZE 11


/*--------------���ڴ�������Ǽ��ٶȼ��������ݽṹ��--------------*/
typedef struct __Mpu6050_Data_ 
{
	short X_data;                       //2���ֽ�
	short Y_data;                       //2���ֽ�
	short Z_data;                       //2���ֽ�
}Mpu6050_Data;


/*��-----------------����λ�����ڷ��ͺͽ������ݵĽṹ��-------------------*/
typedef struct _SEND_DATA_  
{
	unsigned char Frame_Header;//1���ֽ�
	short X_speed;	           //2���ֽ�
	short Y_speed;             //2���ֽ�
	short Z_speed;             //2���ֽ�
	uint16_t Power_Quantity;       //��ص���
	uint16_t Power_Voltage;       //��ص�ѹ
	uint16_t Power_Current;       //��ص���
	uint16_t Power_Temperature;       //����¶�

	uint16_t M1_current;       //< ���1����
	uint16_t M2_current;       //< ���2����
	uint16_t P19_current;       //< 19V��Դ����
	uint16_t P12_current;       //< 12V��Դ����
	uint16_t P5_current;       //< 5V��Դ����

	Mpu6050_Data Accelerometer;//6���ֽ�
	Mpu6050_Data Gyroscope;    //6���ֽ�

	unsigned char Frame_Tail;  //1���ֽ�

}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header;// 1
		float X_speed;	          //4���ֽ�
		float Y_speed;            //4���ֽ�
		float Z_speed;            //4���ֽ�
		unsigned char Frame_Tail;     //1���ֽ�
	}Control_Str;

}RECEIVE_DATA;


/*��-------------��ģģ��usart2�������ݽṹ��--------------*/
typedef struct
{
	unsigned short CH1; //ͨ��1��ֵ
	unsigned short CH2; //ͨ��2��ֵ   ͨ��1��2��������Ҳ�ң����
	unsigned short CH3; //ͨ��3��ֵ
	unsigned short CH4; //ͨ��4��ֵ   ͨ��3��4����������ң����
	unsigned short CH5; //ͨ��5��ֵ   
	unsigned short CH6; //ͨ��6��ֵ    ��Ӧ����SWB
  unsigned short CH7; //ͨ��7��ֵ    ��Ӧ����SWA
  unsigned short CH8; //ͨ��8��ֵ    ��Ӧ����SWC
  unsigned short CH9; //ͨ��9��ֵ    ��Ӧ����SWD
  unsigned short CH10;//ͨ��10��ֵ����Ӧ����VRA�������Ƴ���
  unsigned short CH11;//ͨ��11��ֵ
  unsigned short CH12;//ͨ��12��ֵ
  unsigned short CH13;//ͨ��13��ֵ
  unsigned short CH14;//ͨ��14��ֵ
  unsigned short CH15;//ͨ��15��ֵ
  unsigned short CH16;//ͨ��16��ֵ
	unsigned char ucConnectState;//ң���������������״̬ 0=δ���ӣ�1=��������
}SBUS_CH_Struct;


extern uint8_t uart1_recv_data[USART1_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart1_send_data[USART1_TX_MAXBUFF]; // �������ݻ�����
extern uint32_t uart1_recv_flag;     // ������ɱ�־λ
extern uint32_t uart1_recv_len;      // ���յ����ݳ���
extern uint32_t uart1_send_len;      // ���͵����ݳ���
extern uint32_t uart1_send_flag;     // ������ɱ�־λ


extern uint8_t uart4_recv_data[USART4_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart4_send_data[USART4_TX_MAXBUFF]; // �������ݻ�����
extern uint32_t uart4_recv_flag;     // ������ɱ�־λ
extern uint32_t uart4_recv_len;      // ���յ����ݳ���
extern uint32_t uart4_send_len;      // ���͵����ݳ���
extern uint32_t uart4_send_flag;     // ������ɱ�־λ

extern SBUS_CH_Struct tagSBUS_CH;                    //���ת�����ͨ������
extern unsigned char Uart5_Buffer[Max_BUFF_Len];  //��Ž��յ�������
extern unsigned int  ucRcvCount;                     //��������

extern int g_nVelocity;                              //ch3,������¿���С���ٶ�
extern int g_nDirector;                              //ch1,�ұ����ҿ���С��ת��

extern unsigned char g_ucLightOnFlag;                //���峵���Ƿ�����0
extern float g_fltRecv_Vel_X;                        // ���ڽ��յ����ٶ�����
extern float g_fltRecv_Vel_Y;
extern float g_fltRecv_Vel_Z;

void DATA_task(void *pvParameters);

void Usart5_Init(unsigned int unBound);              //����5��Ϊ��ģ���ݽ���
unsigned char Update_sbus(unsigned char* ucBuf);
void SetReal_Velocity(uint16_t* pdu);                         //���õ�����е��ٶ�
void Set_Director(void);                             //���õ����ת��                          
int Abs_int(int nValue);
int Target_Velocity_get(unsigned short usValue);
int Target_Direct_get(unsigned short usValue);
void Car_Light_Control(void);                         //�������ݽ�������Ӧ����ch10ͨ��������


void Usart3_Init(uint32_t baud);         //usart3��Ϊ����λ��ͨ��ģ��
void USART3_IRQHandler(void);       //�ж�����Ϊ���ݽ��մ���
uint8_t Check_Sum(uint8_t* d, uint8_t Count_Number, uint8_t Mode);
void Data_transition(void);         //����3���͸���λ������ת��

void Printf_MPU9250_Data(void);     //���Դ�ӡimu����Ϣ

void Usart4_Init(uint32_t baud);         //usart4��Ϊ��ȡ�����Ϣģ��


void USART1_Init(uint32_t bound);

void modbus_task_init(void);
void ModBUS_task(void* pvParameters);
#endif

