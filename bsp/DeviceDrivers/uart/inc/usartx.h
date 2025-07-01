/****************************************
Usart2是接收航模遥控器发送过来的数据；
Usart3是与上位机ROS通信的。
****************************************/

#ifndef __USARTX_H
#define __USARTX_H 

#include "stdio.h"
#include "n32g45x_conf.h"
#include "bsp.h"

extern uint16_t* pdu;
extern bool ROS_RecvFlag; 		//ROS消息收到标志位
extern bool Motor_Enable_Flag; 	//电机使能标志位

/*----------------------------任务各参数----------------------------*/
//Usart1任务（串口1）
#define USART1_RX_MAXBUFF   2560 		// 接收数据最大缓冲区
extern uint8_t usart1_recv_data[USART1_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t usart1_recv_len;      // 接收的数据长度
extern uint8_t usart1_recv_flag;     // 接收完成标志位

#define USART1_TX_MAXBUFF   2560 		// 发送数据最大缓冲区
extern uint8_t usart1_send_data[USART1_TX_MAXBUFF]; // 发送数据缓冲区
extern uint8_t usart1_send_len;      // 发送的数据长度
extern uint8_t usart1_send_flag;     // 发送完成标志位

//Usart3任务（RS232）
#define USART3_RX_MAXBUFF   256 		// 接收数据最大缓冲区
extern uint8_t usart3_recv_data[USART3_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t usart3_recv_len;      // 接收的数据长度
extern uint8_t usart3_recv_flag;     // 接收完成标志位

#define USART3_TX_MAXBUFF    50		// 发送数据最大缓冲区
extern uint8_t usart3_send_data[USART3_TX_MAXBUFF]; // 发送数据缓冲区
extern uint8_t usart3_send_len;      // 发送的数据长度
extern uint8_t usart3_send_flag;     // 发送完成标志位

//Uart4任务（ADC）
#define UART4_RX_MAXBUFF   500 		// 接收数据最大缓冲区
extern uint8_t uart4_recv_data[UART4_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t uart4_recv_len;      // 接收的数据长度
extern uint8_t uart4_recv_flag;     // 接收完成标志位

#define UART4_TX_MAXBUFF   25 		// 发送数据最大缓冲区
extern uint8_t uart4_send_data[UART4_TX_MAXBUFF]; // 发送数据缓冲区
extern uint8_t uart4_send_len;      // 发送的数据长度
extern uint8_t uart4_send_flag;     // 发送完成标志位

//Uart5任务（航模）
#define UART5_RX_MAXBUFF   256 		// 接收数据最大缓冲区
extern uint8_t uart5_recv_data[UART5_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t uart5_recv_len;      // 接收的数据长度
extern uint8_t uart5_recv_flag;     // 接收完成标志位

/*----------------------------任务各函数----------------------------*/
//Usart1任务（Modbus）
void Usart1_Init(uint32_t baud);
void Usart1_Dma_Config(void);     		// DMA配置
void DMA1_Channel4_nvic_config(void);
void Modbus_task(void* pvParameters);
void USARTOne_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void modbus_task_init(void);					//初始化Modbus协议栈的工作环境

//Usart3任务（RS232）
void Usart2_Init(uint32_t baud);         //usart2在圆形底盘中代替usart4
void USART2_DMA_Config(void);     		// DMA配置

	
//Usart3任务（RS232）
void Usart3_Init(uint32_t baud);         //usart3作为上下位机通信模块
void Usart3_dma_config(void);     		// DMA配置
void Usart3_Send(void);
void Usart3_Recv(void);
void Data_Assignment(void);         //串口3发送给上位机数据转换

//Usart4任务（ADC）
void Uart4_Init(uint32_t baud);         //uart4作为读取电池信息模块
void Uart4_Dma_Config(void);     		// DMA配置


short ConvertBytesToShort(u8 highbyte,u8 lowbyte);   //上位机传输下来的速度信息解析

//Uart5任务（航模）
void Uart5_Init(unsigned int unBound);              //串口5作为航模数据接收
void Flash_Init(void);
void FLASH_Write_Trigger(void);
void FLASH_Read_Trigger(void);
extern bool FLASH_read_flag;
void Uart5_Dma_Config(void);

/*—-------------航模模块--------------*/
#define Max_BUFF_Len      200          		//一帧数据的长度，25个字节

/*—-------------上下位机通信协议相关宏定义--------------*/
#define FRAME_HEADER 			0X7B  		//发送数据的帧头,"{"
#define FRAME_TAIL 				0X7D      	//发送数据的帧尾,"}"
#define SEND_DATA_SIZE 			24
#define RECEIVE_DATA_SIZE 		17

/*—-------------航模模块--------------*/
#define REMOTE_TYPE_FSi6		1
#define REMOTE_TYPE_HT8A		2
#define REMOTE_TYPE				REMOTE_TYPE_HT8A		//航模遥控器型号切换
#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)
#define RC_MIN_VALUE        	240       	//定义三个速度挡位的基准值,swb控制，ch6通道
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
#define RC_MIN_VALUE        	192       	//定义三个速度挡位的基准值,swb控制，ch6通道
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



#define SPEED_LOW              	500        	//定义三个速度等级
#define SPEED_MIDDLE           	1000
#define SPEED_HIGH            	1300
#define SPEED_DIR_LOW          	300    		//定义三个转弯速度等级
#define SPEED_DIR_MIDDLE       	500
#define SPEED_DIR_HIGH        	800

#define AKM_PRODUCT_NUM			0			// 阿克曼
#define DIFF_PRODUCT_NUM		0			// 差速车(圆形底盘)
#define FOURWHEEL_PRODUCT_NUM	0			// 室外差速四驱车
#define TWOWHEEL_PRODUCT_NUM	0			// 室内差速二驱车
#define TANK_PRODUCT_NUM		0			// 坦克车
#define RC_PRODUCT_NUM			0			// 竞赛小车

#define AKM_VERSION				0x0404		
#define DIFF_VERSION			0x0404		
#define FOURWHEEL_VERSION		0x0404		
#define TWOWHEEL_VERSION		0x0404		
#define TANK_VERSION			0x0404		
#define RC_VERSION				0x0101		

/*—-----------------上下位机串口发送和接收数据的结构体-------------------*/
typedef union   
{
	struct{
	unsigned char Frame_Header;			//1个字节
	unsigned char Motor_Enable_Flag;	//1个字节
	uint8_t car_error_messages;        //车辆错误信息 16
	uint8_t car_state;				//机器人运行状态 15		
	short X_speed;	           			//2个字节
	short Y_speed;	           			//2个字节
	short Z_speed;             			//2个字节
	uint16_t Power_Voltage;       		//电池电压		
	short Power_Current;       		//电池电流
	uint8_t Power_Quantity;      		//电池电量
	char Power_Temperature;   		//电池温度
	uint8_t motor1_node_state;       //电机1节点状态  39
	uint8_t motor2_node_state;       //电机2节点状态	
	uint8_t motor3_node_state;		//电机3节点状态
	uint8_t motor4_node_state;		//电机4节点状态
	uint16_t motor1_error_code;			//电机1错误代码  41
	int16_t motor1_pulse;				//电机1脉冲数低字节
	int16_t motor1_pulse1;				//电机1脉冲数高字节
	uint16_t motor2_error_code;			//电机2错误代码
	int16_t motor2_pulse;				//电机2脉冲数低字节
	int16_t motor2_pulse1;				//电机2脉冲数高字节
	uint16_t motor3_error_code;			//电机3错误代码
	int16_t motor3_pulse;				//电机3脉冲数低字节
	int16_t motor3_pulse1;				//电机3脉冲数高字节
	uint16_t motor4_error_code;			//电机4错误代码
	int16_t motor4_pulse;				//电机4脉冲数低字节
	int16_t motor4_pulse1;				//电机4脉冲数高字节
	uint16_t Odometry1;     			//里程计低字节
	uint16_t Odometry2;     			//里程计高字节
	unsigned char Check_SUM;   			//1个字节
	unsigned char Frame_Tail;  			//1个字节
	}d;	
	unsigned char buffer[USART3_TX_MAXBUFF];  		//数据
}SEND_DATA;

extern unsigned char Receive_Data[RECEIVE_DATA_SIZE];  //存放接收到的数据

extern unsigned char Uart5_Buffer[Max_BUFF_Len];  //存放接收到的数据
extern int ROS_Count;


void DATA_task(void *pvParameters);

void Uart5_Init(unsigned int unBound);              //串口5作为航模数据接收
uint32_t Abs_int(int nValue);
void JTAG_Set(uint8_t mode);

unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode);
void Data_transition(void);         //串口3发送给上位机数据转换

void Jump_To_BOOT(void);
void Pdu_Init(void);
static u16 MDBcrc(u8* dataBuf, u8 len);
extern  u8 ros_motor_en;

#endif

