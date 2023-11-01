#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define MODBUS_STK_SIZE   128 	//�����ջ��С
#define MODBUS_TASK_PRIO  4   
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define USART1_RX_MAXBUFF   256 		// ����������󻺳���
#define USART1_TX_MAXBUFF   256 		// ����������󻺳���
extern uint8_t uart1_recv_data[USART1_RX_MAXBUFF]; // �������ݻ�����
extern uint8_t uart1_send_data[USART1_TX_MAXBUFF]; // �������ݻ�����
extern uint32_t uart1_recv_flag;     // ������ɱ�־λ
extern uint32_t uart1_recv_len;      // ���յ����ݳ���
extern uint32_t uart1_send_len;      // ���͵����ݳ���
extern uint32_t uart1_send_flag;     // ������ɱ�־λ
void USART1_Init(uint32_t bound);

void modbus_task_init(void);
void modbus_task(void* pvParameters);
#endif


