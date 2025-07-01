/****************************************
Usart2�ǽ��պ�ģң�������͹��������ݣ�
Usart3������λ��ROSͨ�ŵġ�
****************************************/

#ifndef __USARTX_H
#define __USARTX_H 

#include "stdio.h"
#include "n32g45x_conf.h"
#include "bsp.h"

extern uint16_t* pdu;
extern bool ROS_RecvFlag; 		//ROS��Ϣ�յ���־λ
extern bool Motor_Enable_Flag; 	//���ʹ�ܱ�־λ

/*----------------------------���������----------------------------*/
//Usart1���񣨴���1��
#define USART1_RX_MAXBUFF   2560 		// ����������󻺳���
extern uint8_t usart1_recv_data[USART1_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t usart1_recv_len;      // ���յ����ݳ���
extern uint8_t usart1_recv_flag;     // ������ɱ�־λ

#define USART1_TX_MAXBUFF   2560 		// ����������󻺳���
extern uint8_t usart1_send_data[USART1_TX_MAXBUFF]; // �������ݻ�����
extern uint8_t usart1_send_len;      // ���͵����ݳ���
extern uint8_t usart1_send_flag;     // ������ɱ�־λ

//Usart3����RS232��
#define USART3_RX_MAXBUFF   256 		// ����������󻺳���
extern uint8_t usart3_recv_data[USART3_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t usart3_recv_len;      // ���յ����ݳ���
extern uint8_t usart3_recv_flag;     // ������ɱ�־λ

#define USART3_TX_MAXBUFF    50		// ����������󻺳���
extern uint8_t usart3_send_data[USART3_TX_MAXBUFF]; // �������ݻ�����
extern uint8_t usart3_send_len;      // ���͵����ݳ���
extern uint8_t usart3_send_flag;     // ������ɱ�־λ

//Uart4����ADC��
#define UART4_RX_MAXBUFF   500 		// ����������󻺳���
extern uint8_t uart4_recv_data[UART4_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart4_recv_len;      // ���յ����ݳ���
extern uint8_t uart4_recv_flag;     // ������ɱ�־λ

#define UART4_TX_MAXBUFF   25 		// ����������󻺳���
extern uint8_t uart4_send_data[UART4_TX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart4_send_len;      // ���͵����ݳ���
extern uint8_t uart4_send_flag;     // ������ɱ�־λ

//Uart5���񣨺�ģ��
#define UART5_RX_MAXBUFF   256 		// ����������󻺳���
extern uint8_t uart5_recv_data[UART5_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart5_recv_len;      // ���յ����ݳ���
extern uint8_t uart5_recv_flag;     // ������ɱ�־λ

/*----------------------------���������----------------------------*/
//Usart1����Modbus��
void Usart1_Init(uint32_t baud);
void Usart1_Dma_Config(void);     		// DMA����
void DMA1_Channel4_nvic_config(void);
void Modbus_task(void* pvParameters);
void USARTOne_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void modbus_task_init(void);					//��ʼ��ModbusЭ��ջ�Ĺ�������

//Usart3����RS232��
void Usart2_Init(uint32_t baud);         //usart2��Բ�ε����д���usart4
void USART2_DMA_Config(void);     		// DMA����

	
//Usart3����RS232��
void Usart3_Init(uint32_t baud);         //usart3��Ϊ����λ��ͨ��ģ��
void Usart3_dma_config(void);     		// DMA����
void Usart3_Send(void);
void Usart3_Recv(void);
void Data_Assignment(void);         //����3���͸���λ������ת��

//Usart4����ADC��
void Uart4_Init(uint32_t baud);         //uart4��Ϊ��ȡ�����Ϣģ��
void Uart4_Dma_Config(void);     		// DMA����


short ConvertBytesToShort(u8 highbyte,u8 lowbyte);   //��λ�������������ٶ���Ϣ����

//Uart5���񣨺�ģ��
void Uart5_Init(unsigned int unBound);              //����5��Ϊ��ģ���ݽ���
void Flash_Init(void);
void FLASH_Write_Trigger(void);
void FLASH_Read_Trigger(void);
extern bool FLASH_read_flag;
void Uart5_Dma_Config(void);

/*��-------------��ģģ��--------------*/
#define Max_BUFF_Len      200          		//һ֡���ݵĳ��ȣ�25���ֽ�

/*��-------------����λ��ͨ��Э����غ궨��--------------*/
#define FRAME_HEADER 			0X7B  		//�������ݵ�֡ͷ,"{"
#define FRAME_TAIL 				0X7D      	//�������ݵ�֡β,"}"
#define SEND_DATA_SIZE 			24
#define RECEIVE_DATA_SIZE 		17

/*��-------------��ģģ��--------------*/
#define REMOTE_TYPE_FSi6		1
#define REMOTE_TYPE_HT8A		2
#define REMOTE_TYPE				REMOTE_TYPE_HT8A		//��ģң�����ͺ��л�
#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)
#define RC_MIN_VALUE        	240       	//���������ٶȵ�λ�Ļ�׼ֵ,swb���ƣ�ch6ͨ��
#define RC_BASE_VALUE       	1023
#define RC_MAX_VALUE        	1807
#define RC_GEARS_DIFFERENCE     784
#define Forward_value			rc_ch3_value
#define Turn_value				rc_ch1_value
#define GearPosition_value		rc_ch6_value
#define Enable_value			rc_ch7_value
#define EmergencyStop_value		rc_ch9_value
#define Torque_value			rc_ch10_value

#elif(REMOTE_TYPE == REMOTE_TYPE_HT8A)
#define RC_MIN_VALUE        	192       	//���������ٶȵ�λ�Ļ�׼ֵ,swb���ƣ�ch6ͨ��
#define RC_BASE_VALUE       	992
#define RC_MAX_VALUE        	1792
#define RC_GEARS_DIFFERENCE     800
#define Forward_value			rc_ch3_value
#define Turn_value				rc_ch1_value
#define GearPosition_value		rc_ch5_value
#define Enable_value			rc_ch6_value
#define EmergencyStop_value		rc_ch7_value
#define Torque_value			rc_ch8_value

#endif



#define SPEED_LOW              	500        	//���������ٶȵȼ�
#define SPEED_MIDDLE           	1000
#define SPEED_HIGH            	1300
#define SPEED_DIR_LOW          	300    		//��������ת���ٶȵȼ�
#define SPEED_DIR_MIDDLE       	500
#define SPEED_DIR_HIGH        	800

#define AKM_PRODUCT_NUM			0			// ������
#define DIFF_PRODUCT_NUM		0			// ���ٳ�(Բ�ε���)
#define FOURWHEEL_PRODUCT_NUM	0			// �������������
#define TWOWHEEL_PRODUCT_NUM	0			// ���ڲ��ٶ�����
#define TANK_PRODUCT_NUM		0			// ̹�˳�
#define RC_PRODUCT_NUM			0			// ����С��

#define AKM_VERSION				0x0404		
#define DIFF_VERSION			0x0404		
#define FOURWHEEL_VERSION		0x0404		
#define TWOWHEEL_VERSION		0x0404		
#define TANK_VERSION			0x0404		
#define RC_VERSION				0x0101		

/*��-----------------����λ�����ڷ��ͺͽ������ݵĽṹ��-------------------*/
typedef union   
{
	struct{
	unsigned char Frame_Header;			//1���ֽ�
	unsigned char Motor_Enable_Flag;	//1���ֽ�
	uint8_t car_error_messages;        //����������Ϣ 16
	uint8_t car_state;				//����������״̬ 15		
	short X_speed;	           			//2���ֽ�
	short Y_speed;	           			//2���ֽ�
	short Z_speed;             			//2���ֽ�
	uint16_t Power_Voltage;       		//��ص�ѹ		
	short Power_Current;       		//��ص���
	uint8_t Power_Quantity;      		//��ص���
	char Power_Temperature;   		//����¶�
	uint8_t motor1_node_state;       //���1�ڵ�״̬  39
	uint8_t motor2_node_state;       //���2�ڵ�״̬	
	uint8_t motor3_node_state;		//���3�ڵ�״̬
	uint8_t motor4_node_state;		//���4�ڵ�״̬
	uint16_t motor1_error_code;			//���1�������  41
	int16_t motor1_pulse;				//���1���������ֽ�
	int16_t motor1_pulse1;				//���1���������ֽ�
	uint16_t motor2_error_code;			//���2�������
	int16_t motor2_pulse;				//���2���������ֽ�
	int16_t motor2_pulse1;				//���2���������ֽ�
	uint16_t motor3_error_code;			//���3�������
	int16_t motor3_pulse;				//���3���������ֽ�
	int16_t motor3_pulse1;				//���3���������ֽ�
	uint16_t motor4_error_code;			//���4�������
	int16_t motor4_pulse;				//���4���������ֽ�
	int16_t motor4_pulse1;				//���4���������ֽ�
	uint16_t Odometry1;     			//��̼Ƶ��ֽ�
	uint16_t Odometry2;     			//��̼Ƹ��ֽ�
	unsigned char Check_SUM;   			//1���ֽ�
	unsigned char Frame_Tail;  			//1���ֽ�
	}d;	
	unsigned char buffer[USART3_TX_MAXBUFF];  		//����
}SEND_DATA;

extern unsigned char Receive_Data[RECEIVE_DATA_SIZE];  //��Ž��յ�������

extern unsigned char Uart5_Buffer[Max_BUFF_Len];  //��Ž��յ�������
extern int ROS_Count;


void DATA_task(void *pvParameters);

void Uart5_Init(unsigned int unBound);              //����5��Ϊ��ģ���ݽ���
uint32_t Abs_int(int nValue);
void JTAG_Set(uint8_t mode);

unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode);
void Data_transition(void);         //����3���͸���λ������ת��

void Jump_To_BOOT(void);
void Pdu_Init(void);
static u16 MDBcrc(u8* dataBuf, u8 len);
extern  u8 ros_motor_en;

#endif

