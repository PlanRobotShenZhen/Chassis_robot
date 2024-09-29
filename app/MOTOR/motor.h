#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "can.h"
#include "485_address.h"
#include "n32g45x.h"
extern int Servo_pulse;								//小车前方电机脉冲

void Motor_task(void* pvParameters);
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
