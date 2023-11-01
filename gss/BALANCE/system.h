#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "n32g45x.h"                    // Device header
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*��������ͷ�ļ�*/
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
#include "robot_select_init.h"    // ���ͷ�ļ����涨��Ķ��ǻ�е�ṹ��صĲ��������Ǻ궨��Ĳ���

/*һЩC�⺯�������ͷ�ļ�*/
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"


#define CONTROL_DELAY		1000 // �����ʵ��ʱ����10��,Ϊ������������ȡ�����ǵ����
#define RATE_1_HZ		  1      // Ƶ�ʴ������freeRTOS��ÿ�������ִ��Ƶ��
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


// ö�ٶ��弸��С��������
enum CarMode
{
	UNKNOW = -1,
	Mec_Car = 0,        // �����ĸ��С��
	Omni_Car,           // ����С��
	Akm_Car,            // ������
	Diff_Car,           // ���ٳ�
	FourWheel_Car,      // ������
	Tank_Car            // ̹�˳�
};

 
// ÿ�������ز�������Ŀ���ٶȺͷ����ٶ�
struct Motor_parameter
{
	int nTarget_Velocity;        //��λr/min,Ϊ�˱������������д��������
	int nFeedback_Velocity;
	float fltTarget_velocity;    //������λ��ͨ�ţ�����������ӵ��ٶ�m/s
	float fltFeedBack_Velocity;
};


// ö�ٶ���С���Ŀ��Ʒ�ʽ
enum ENUM_CarControl_Mode
{
	CONTROL_MODE_UNKNOW = 0,    // δ֪����ģʽ
	CONTROL_MODE_REMOTE = 1,    // ��ģ���Ʒ�ʽ
	CONTROL_MODE_ROS    = 2,    // ��λ������ģʽ
	CONTROL_MODE_UART   = 3,    // ���ڿ���ģʽ
	CONTROL_MODE_OTHER          // ��������ģʽ
};
//��ģ���ýṹ��
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
//��ģ�����ṹ��ָ��
extern Remote_Control_struct *rc_ptr;
//��������ṹ��
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
////С���ٶȽṹ��
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
/*--------------- С����е�ṹ����-----------------------*/
extern float Wheel_perimeter; //�����ܳ�����λ���ף�
extern float Wheel_spacing;   //�������־� ����λ���ף�
extern float Axle_spacing;    //����ǰ�������
extern float Omni_turn_radiaus; //ȫ����ת��뾶


/*--------------- ��ʼ��ʱ����ر�־λ-----------------------*/
extern enum CarMode g_emCarMode;                   //������ѡ�ͱ�־λ
extern unsigned char Flag_Stop;                    //������ʹ�ܱ�־λ��1����رյ��
extern enum ENUM_CarControl_Mode g_eControl_Mode;  //�����˵Ŀ��Ʒ�ʽ
extern unsigned char g_ucRemote_Flag;              //��ģ������־λ
extern unsigned char g_ucRos_Flag;                 // ROS��λ�������־λ 


extern struct Motor_parameter  MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
void Soft_Reset(void);  // �ֶ������λ
void RCC_HSE_Config(unsigned int unDiv, unsigned int unPllm);  // ����ʱ��ѡ���ⲿʱ��Դ

void systemInit(void);
extern uint16_t* getPDUData(void);
#endif /* __SYSTEM_H */
