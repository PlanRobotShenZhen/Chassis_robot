#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"


#define BALANCE_TASK_PRIO		11     //�������ȼ�
#define BALANCE_STK_SIZE 		256   //�����ջ��С
#define DIRECTOR_BASE       784   //Z����ٶȵĻ�׼

//ȫ���ֻ�������ѧģ�Ͳ���
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)  
/* 485�궨���ַ */
#define CONTROL_MODE_ADDR  156    // �����˵Ŀ��Ʒ�ʽ485��ַ
//���ƽ�����ƺ������
struct Smooth_Control
{
	float Vx;
	float Vy;
	float Vz;
	
};

extern struct Smooth_Control tagSmooth_control;  //�����ķ����Ҫ�õ���С���ٶ�ƽ������
extern float Move_X,Move_Y,Move_Z;   //С����������ٶ�


/*----------------------���Ŀ��ƺ���--------------------------*/
void Balance_task(void *pvParameters);           //������
void Remote_Control(void);                       //��ģң�����������ݴ���
void Ros_Control(void);                          //��ģң�����������ݴ���
void Drive_Motor(float Vx,float Vy,float Vz);    //С���˶�ģ�ͣ��������������ٶ�
																							   //����4������ٶ�
void Set_MotorVelocity(int nMotorLB,int nMotorLF, int nMotorRF, int nMotorRB);    
void Get_Motor_Velocity(void);                   //��ȡ����ٶȷ���ֵ��r/min
void Smooth_control(float vx,float vy,float vz); //ȫ����С���ٶ�ƽ��
unsigned char Turn_Off(void);                    //���ݵ�ص���������
void Update_Gyroscope(void);                     //�������������
float RotateToSpeedVelocity(int nRotateSpeed);   //���ת��ת��Ϊ�������ٶȣ�m/s
int SpeedVelocityToRotate(float fltSpeed);       //�����ٶ�ת��Ϊ���ת��r/min


/*----------------------�������ܺ���--------------------------*/
u32 myabs(long int a);
float float_abs(float insert);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

#endif  

