#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define MODBUS_STK_SIZE   128 	//任务堆栈大小
#define MODBUS_TASK_PRIO  4   
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define USART1_RX_MAXBUFF   256 		// 接收数据最大缓冲区
#define USART1_TX_MAXBUFF   256 		// 接收数据最大缓冲区
extern uint8_t uart1_recv_data[USART1_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t uart1_send_data[USART1_TX_MAXBUFF]; // 发送数据缓冲区
extern uint32_t uart1_recv_flag;     // 接收完成标志位
extern uint32_t uart1_recv_len;      // 接收的数据长度
extern uint32_t uart1_send_len;      // 发送的数据长度
extern uint32_t uart1_send_flag;     // 发送完成标志位
void USART1_Init(uint32_t bound);

void modbus_task_init(void);
void modbus_task(void* pvParameters);
#endif


