#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "can.h"
#include "485_address.h"
#include "n32g45x.h"
#define   ENCODERDATAABNORMAL     			0x0A337306		//编码器异常
#define   OVERVOLTAGE     					0x04003210		//过压
#define   HARDWAREOVERVCURRENT				0x02012312		//硬件过流
#define   SOFTWAREOVERVCURRENT     			0x02072311		//软件过流
#define   NOTOROVERLOADED    				0x06203230		//电机过载
#define   FLYINGCAR							0x0234FF00		//飞车	
#define   WATCHDOGERROR     				0x01056320		//内部程序异常触发看门狗 
#define   CANCORRESPONDENCE     			0x0D038130		//CAN 通信连接中断
#define   UNDERVOLTAGE						0x04103220		//欠压
#define   POSITIVEOVERTRAVELWARNING     	0x09505443		// 正向超程警告
#define   NEGATIVEOVERTRAVELWARNING     	0x09525444		// 负向超程警告
#define   POSITIONDEVIATIONLARGE			0x0B008611		//位置偏差过大
#define   EMERGENCYSHUTDOWN     			0x09005442		// 紧急停机		
#define   ENCODERCOUNTERROR	     			0x07337306		// 编码器多圈计数错误 
#define   ENCODERCOUNTOVERFLOW				0x07357306		//编码器多圈计数溢出  
extern int Servo_pulse;								//小车前方电机脉冲

void Motor_task(void* pvParameters);
void Motor_en_task(void* pvParameters);
void Motor_Init(int count);
void Detect_Motor_Status(void);            //是否出重新使能电机

void Set_MotorVelocity(void); 
enum enum_car_running_state Judge_CarState(uint16_t Car_Error_Label, bool Soft_JT_Flag); 
void InitMotorParameters(void);
short limit_value(short value, short max_value);
extern uint32_t Odom_distance_mm;
extern float TireCircumference;
extern float MotorSpeedToLineSpeed;
extern float LineSpeedToMotorSpeed;
extern float MotorSpeedZLACToPulse;
extern float MotorPulseWANZEToMotorSpeed;
bool CanIRQProcess(CAN_Module* CANx);
#endif
