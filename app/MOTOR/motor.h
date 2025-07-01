#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "can.h"
#include "485_address.h"
#include "n32g45x.h"
#define   ENCODERDATAABNORMAL     			0x0A337306		//�������쳣
#define   OVERVOLTAGE     					0x04003210		//��ѹ
#define   HARDWAREOVERVCURRENT				0x02012312		//Ӳ������
#define   SOFTWAREOVERVCURRENT     			0x02072311		//�������
#define   NOTOROVERLOADED    				0x06203230		//�������
#define   FLYINGCAR							0x0234FF00		//�ɳ�	
#define   WATCHDOGERROR     				0x01056320		//�ڲ������쳣�������Ź� 
#define   CANCORRESPONDENCE     			0x0D038130		//CAN ͨ�������ж�
#define   UNDERVOLTAGE						0x04103220		//Ƿѹ
#define   POSITIVEOVERTRAVELWARNING     	0x09505443		// ���򳬳̾���
#define   NEGATIVEOVERTRAVELWARNING     	0x09525444		// ���򳬳̾���
#define   POSITIONDEVIATIONLARGE			0x0B008611		//λ��ƫ�����
#define   EMERGENCYSHUTDOWN     			0x09005442		// ����ͣ��		
#define   ENCODERCOUNTERROR	     			0x07337306		// ��������Ȧ�������� 
#define   ENCODERCOUNTOVERFLOW				0x07357306		//��������Ȧ�������  
extern int Servo_pulse;								//С��ǰ���������

void Motor_task(void* pvParameters);
void Motor_en_task(void* pvParameters);
void Motor_Init(int count);
void Detect_Motor_Status(void);            //�Ƿ������ʹ�ܵ��

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
