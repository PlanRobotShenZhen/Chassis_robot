#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "n32g45x.h"                    // Device header
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*外设的相关头文件*/
#include "sys.h"
#include "delay.h"
#include "balance.h"
#include "led.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "ioi2c.h"
#include "mpu9250.h"
#include "show.h"								   
#include "robot_select_init.h"    // 这个头文件里面定义的都是机械结构相关的参数，都是宏定义的参数

/*一些C库函数的相关头文件*/
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"


#define CONTROL_DELAY		1000 // 换算成实际时间是10秒,为了消除后续读取陀螺仪的零点
#define RATE_1_HZ		  1      // 频率代表的是freeRTOS中每个任务的执行频率
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000


// 枚举定义几种小车的类型
enum CarMode
{
	UNKNOW = -1,
	Mec_Car = 0,        // 麦克那母轮小车
	Omni_Car,           // 麦轮小车
	Akm_Car,            // 阿克曼
	Diff_Car,           // 差速车
	FourWheel_Car,      // 四驱车
	Tank_Car            // 坦克车
};

 
// 每个电机相关参数，有目标速度和反馈速度
struct Motor_parameter
{
	int nTarget_Velocity;        //单位r/min,为了便于做电机控制写入驱动器
	int nFeedback_Velocity;
	float fltTarget_velocity;    //做上下位机通信，传输的是轮子的速度m/s
	float fltFeedBack_Velocity;
};


// 枚举定义小车的控制方式
enum ENUM_CarControl_Mode
{
	CONTROL_MODE_UNKNOW = 0,    // 未知控制模式
	CONTROL_MODE_REMOTE = 1,    // 航模控制方式
	CONTROL_MODE_ROS    = 2,    // 上位机控制模式
	CONTROL_MODE_UART   = 3,    // 串口控制模式
	CONTROL_MODE_OTHER          // 其他控制模式
};
//航模设置结构体
typedef struct REMOTE_CONTROL
{
	uint16_t turn_off_remote;
	uint16_t turn_on_remote;
	uint16_t vel_base_value;
	uint16_t dir_base_value;
	uint16_t limit_max_val;
	uint16_t limit_min_val;
	uint16_t speed_level1;
	uint16_t speed_level2;
	uint16_t speed_level3;
	uint16_t speed_low;
	uint16_t speed_middle;
	uint16_t speed_high;
	uint16_t speed_dir_low;
	uint16_t speed_dir_middle;
	uint16_t speed_dir_high;
	uint16_t light_base;
	uint16_t light_max;
	uint16_t light_min;
}Remote_Control_struct;
//航模参数结构体指针
extern Remote_Control_struct *rc_ptr;
//电机参数结构体
typedef struct motor_struct
{
	uint16_t motor_state;
	uint16_t motor_per;
	uint16_t reduction_ratio;
	uint16_t can_id;
	uint16_t can_baud_rate;
	uint16_t heartbeat_time;
}Motor_struct;
extern Motor_struct *motorA_ptr;
extern Motor_struct *motorB_ptr;
extern Motor_struct *motorC_ptr;
extern Motor_struct *motorD_ptr;
////小车速度结构体
//typedef struct robot_speed
//{
//	float max_lin_speed;
//	float min_lin_speed;
//	float max_ang_speed;
//	float min_ang_speed;
//	float tar_lin_speed;
//	float tar_ang_speed;
//}Robot_speed;
//extern Robot_speed *robot_speed_ptr;
/*--------------- 小车机械结构参数-----------------------*/
extern float Wheel_perimeter; //轮子周长（单位：米）
extern float Wheel_spacing;   //主动轮轮距 （单位：米）
extern float Axle_spacing;    //麦轮前后轴轴距
extern float Omni_turn_radiaus; //全向轮转弯半径


/*--------------- 初始化时的相关标志位-----------------------*/
extern enum CarMode g_emCarMode;                   //机器人选型标志位
extern unsigned char Flag_Stop;                    //机器人使能标志位，1代表关闭电机
extern enum ENUM_CarControl_Mode g_eControl_Mode;  //机器人的控制方式
extern unsigned char g_ucRemote_Flag;              //航模开启标志位
extern unsigned char g_ucRos_Flag;                 // ROS上位机进入标志位 


extern struct Motor_parameter  MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
void Soft_Reset(void);  // 手动软件复位
void RCC_HSE_Config(unsigned int unDiv, unsigned int unPllm);  // 配置时钟选用外部时钟源

void systemInit(void);
extern uint16_t* getPDUData(void);
#endif /* __SYSTEM_H */
