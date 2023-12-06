/****************************************
Usart2是接收航模遥控器发送过来的数据；
Usart3是与上位机ROS通信的。
****************************************/

#ifndef __USARTX_H
#define __USARTX_H 

#include "stdio.h"
#include "n32g45x_conf.h"
#include "bsp.h"




#define USART1_RX_MAXBUFF   2560 		// 接收数据最大缓冲区
#define USART1_TX_MAXBUFF   256 		// 接收数据最大缓冲区
#define USART4_RX_MAXBUFF 500
#define USART4_TX_MAXBUFF 25
/*—-------------航模模块--------------*/
#define Max_BUFF_Len      200          //一帧数据的长度，25个字节
#define VEL_BASE_VALUE   1023         //ch3通道基准值，速度
#define DIR_BASE_VALUE   1023         //ch1通道基准值，方向
#define LIMIT_MAX_VAL    1807         //限幅值，max
#define LIMIT_MIN_VAL    240          //限幅值，min
#define MOTOR_MULTI       3           //速度倍率

#define SPEED_LEVEL1        240       //定义三个速度挡位的基准值,swb控制，ch6通道
#define SPEED_LEVEL2        1023
#define SPEED_LEVEL3        1807

#define SPEED_LOW              500        //定义三个速度等级
#define SPEED_MIDDLE           1000
#define SPEED_HIGH            1300
#define SPEED_DIR_LOW          300    //定义三个转弯速度等级
#define SPEED_DIR_MIDDLE       500
#define SPEED_DIR_HIGH        800

#define TURN_OFF_REMOTE      240      //定义航模开关，ch7通道
#define TURN_ON_REMOTE       1807  

#define LIGHT_BASE           1017     //定义车灯的三个基准值
#define LIGHT_MAX            1807
#define LIGHT_MIN            240




extern int g_nLastVelocity;           //存放当前的速度和上一刻的速度
extern int g_nCurrentVelocity;

/*—-------------上下位机通信协议相关宏定义--------------*/
#define FRAME_HEADER 0X7B             //发送数据的帧头,"{"
#define FRAME_TAIL 0X7D               //发送数据的帧尾,"}"
#define SEND_DATA_SIZE 34
#define RECEIVE_DATA_SIZE 11


/*--------------用于存放陀螺仪加速度计三轴数据结构体--------------*/
typedef struct __Mpu6050_Data_ 
{
	short X_data;                       //2个字节
	short Y_data;                       //2个字节
	short Z_data;                       //2个字节
}Mpu6050_Data;


/*—-----------------上下位机串口发送和接收数据的结构体-------------------*/
typedef struct _SEND_DATA_  
{
	unsigned char Frame_Header;//1个字节
	short X_speed;	           //2个字节
	short Y_speed;             //2个字节
	short Z_speed;             //2个字节
	uint16_t Power_Quantity;       //电池电量
	uint16_t Power_Voltage;       //电池电压
	uint16_t Power_Current;       //电池电流
	uint16_t Power_Temperature;       //电池温度

	uint16_t M1_current;       //< 电机1电流
	uint16_t M2_current;       //< 电机2电流
	uint16_t P19_current;       //< 19V电源电流
	uint16_t P12_current;       //< 12V电源电流
	uint16_t P5_current;       //< 5V电源电流

	Mpu6050_Data Accelerometer;//6个字节
	Mpu6050_Data Gyroscope;    //6个字节

	unsigned char Frame_Tail;  //1个字节

}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header;// 1
		float X_speed;	          //4个字节
		float Y_speed;            //4个字节
		float Z_speed;            //4个字节
		unsigned char Frame_Tail;     //1个字节
	}Control_Str;

}RECEIVE_DATA;


/*—-------------航模模块usart2接收数据结构体--------------*/
typedef struct
{
	unsigned short CH1; //通道1数值
	unsigned short CH2; //通道2数值   通道1，2代表的是右侧遥控器
	unsigned short CH3; //通道3数值
	unsigned short CH4; //通道4数值   通道3，4代表的是左侧遥控器
	unsigned short CH5; //通道5数值   
	unsigned short CH6; //通道6数值    对应的是SWB
  unsigned short CH7; //通道7数值    对应的是SWA
  unsigned short CH8; //通道8数值    对应的是SWC
  unsigned short CH9; //通道9数值    对应的是SWD
  unsigned short CH10;//通道10数值，对应的是VRA，来控制车灯
  unsigned short CH11;//通道11数值
  unsigned short CH12;//通道12数值
  unsigned short CH13;//通道13数值
  unsigned short CH14;//通道14数值
  unsigned short CH15;//通道15数值
  unsigned short CH16;//通道16数值
	unsigned char ucConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CH_Struct;


extern uint8_t uart1_recv_data[USART1_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t uart1_send_data[USART1_TX_MAXBUFF]; // 发送数据缓冲区
extern uint32_t uart1_recv_flag;     // 接收完成标志位
extern uint32_t uart1_recv_len;      // 接收的数据长度
extern uint32_t uart1_send_len;      // 发送的数据长度
extern uint32_t uart1_send_flag;     // 发送完成标志位


extern uint8_t uart4_recv_data[USART4_RX_MAXBUFF]; // 接收数据缓冲区
extern uint8_t uart4_send_data[USART4_TX_MAXBUFF]; // 发送数据缓冲区
extern uint32_t uart4_recv_flag;     // 接收完成标志位
extern uint32_t uart4_recv_len;      // 接收的数据长度
extern uint32_t uart4_send_len;      // 发送的数据长度
extern uint32_t uart4_send_flag;     // 发送完成标志位

extern SBUS_CH_Struct tagSBUS_CH;                    //存放转换后的通道数据
extern unsigned char Uart5_Buffer[Max_BUFF_Len];  //存放接收到的数据
extern unsigned int  ucRcvCount;                     //索引计数

extern int g_nVelocity;                              //ch3,左边上下控制小车速度
extern int g_nDirector;                              //ch1,右边左右控制小车转向

extern unsigned char g_ucLightOnFlag;                //定义车灯是否开启，0
extern float g_fltRecv_Vel_X;                        // 串口接收到的速度数据
extern float g_fltRecv_Vel_Y;
extern float g_fltRecv_Vel_Z;

void DATA_task(void *pvParameters);

void Usart5_Init(unsigned int unBound);              //串口5作为航模数据接收
unsigned char Update_sbus(unsigned char* ucBuf);
void SetReal_Velocity(uint16_t* pdu);                         //设置电机运行的速度
void Set_Director(void);                             //设置电机的转向                          
int Abs_int(int nValue);
int Target_Velocity_get(unsigned short usValue);
int Target_Direct_get(unsigned short usValue);
void Car_Light_Control(void);                         //车灯数据解析，对应的是ch10通道的数据


void Usart3_Init(uint32_t baud);         //usart3作为上下位机通信模块
void USART3_IRQHandler(void);       //中断中作为数据接收处理
uint8_t Check_Sum(uint8_t* d, uint8_t Count_Number, uint8_t Mode);
void Data_transition(void);         //串口3发送给上位机数据转换

void Printf_MPU9250_Data(void);     //调试打印imu的信息

void Usart4_Init(uint32_t baud);         //usart4作为读取电池信息模块


void USART1_Init(uint32_t bound);

void modbus_task_init(void);
void ModBUS_task(void* pvParameters);
#endif

