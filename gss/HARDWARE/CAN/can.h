#ifndef __can__h
#define __can__h
#include "n32g45x.h"                    // Device header


#define CAN_RX0_INT_ENABLE 1   //使用中断标识符


extern int g_nMotorLF_config;
extern int g_nMotorLB_config;  
extern int g_nMotorRF_config;
extern int g_nMotorRB_config;  //左右两侧电机配置变量


extern int g_nReInitMotor_LB;   //处理驱动器断电而开发板未断电，重新初始化驱动器的问题
extern int g_nReInitMotor_LF;   //为1代表需要重新初始化，为0则代表不需要重新初始化
extern int g_nReInitMotor_RF;
extern int g_nReInitMotor_RB;


extern int g_nHeart_count_LB;    //设置了心跳包，在规定次数内没产生心跳，则认为从机下线
extern int g_nHeart_count_LF;    //将其状态设置为STATE_STOP
extern int g_nHeart_count_RF;
extern int g_nHeart_count_RB;

extern int g_nHeart_Lastcount_LB;  //记录上次进入心跳包的计数值
extern int g_nHeart_Lastcount_LF;
extern int g_nHeart_Lastcount_RF;
extern int g_nHeart_Lastcount_RB;


//定义存放反馈速度的共用体
union URcv_Vel_Data{
		int nVelcity;
		unsigned char ucData[4];
};

// 定义驱动器故障接收的共用体
union URcv_ERROR_Data{
		unsigned short usError;
		unsigned char ucData[2];
};


//定义心跳包状态
enum ENUM_HEART_STAT
{
		STATE_UNKNOW      = 0x00,   //未知状态
		STATE_STOP        = 0x04,		//停止状态
		STATE_START       = 0x05,		//操作状态
		STATE_PRE_OPERATE = 0x7F		//预操作状态
};

// 定义驱动器的故障状态
enum ENUM_ERROR_STATE
{
	ERROR_NONE              = 0x00,  // 无错误
	ERROR_OVER_VALUE        = 0x01,  // 过压
	ERROR_LESS_VALUE        = 0x02,  // 欠压
	ERROR_OVER_CURRENT      = 0x04,  // 过流
	ERROR_OVER_LOAD         = 0x08,  // 过载
	ERROR_OVERDIFF_CURRENT  = 0x10,  // 电流超差
	ERROR_OVERDIFF_ENCODER  = 0x20,  // 编码器超差
	ERROR_OVERDIFF_VELOCITY = 0x40,  // 速度超差
	ERROR_REF_VALUE         = 0x80,  // 参考电压出错
	ERROR_EEPROM            = 0x100, // EEPROM读写错误
	ERROR_HALL              = 0x200  // 霍尔出错
};

extern union URcv_Vel_Data uVelLF;
extern union URcv_Vel_Data uVelLB;
extern union URcv_Vel_Data uVelRF;
extern union URcv_Vel_Data uVelRB;  //存放左右实时速度的变量

extern union URcv_ERROR_Data uError; // 接收驱动器的故障状态

extern enum ENUM_HEART_STAT eLF_Motor_Heart;
extern enum ENUM_HEART_STAT eLB_Motor_Heart;
extern enum ENUM_HEART_STAT eRF_Motor_Heart; 
extern enum ENUM_HEART_STAT eRB_Motor_Heart;   //左右电机心跳包

extern enum ENUM_ERROR_STATE eLF_Motor_Error;
extern enum ENUM_ERROR_STATE eLB_Motor_Error;
extern enum ENUM_ERROR_STATE eRF_Motor_Error;
extern enum ENUM_ERROR_STATE eRB_Motor_Error;   // 驱动器故障码读取

void Can_Driver_Init(void);

#endif

