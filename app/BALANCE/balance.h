#ifndef __BALANCE_H
#define __BALANCE_H			

#include "stdint.h"

#define BALANCE_TASK_PRIO		11     //�������ȼ�
#define BALANCE_STK_SIZE 		256   //�����ջ��С
#define DIRECTOR_BASE       784   //Z����ٶȵĻ�׼

//ȫ���ֻ�������ѧģ�Ͳ���
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)  

//���׮��ز���
#define LED_LEFT 			Light_Q
#define LED_RIGHT 			Light_H
#define MCU_INF_TX 			exio_output.bit.RGB_G
#define MCU_RELAY2 			exio_output.bit.RGB_B
#define IrDA_TX 			MCU_INF_TX
//������ز���
#define MCU_FAN1			exio_output.bit.Light_Y
#define MCU_FAN2			exio_output.bit.Light_Z
#define FAN1				MCU_FAN1
#define FAN2				MCU_FAN2
//���缫��·���
#define MCU_CH_DET			exio_input.bit.CS2_Ttig
//���ƽ�����ƺ������
struct Smooth_Control
{
	float Vx;
	float Vy;
	float Vz;
	
};

typedef struct {
	uint32_t t_RGB_G;
	uint32_t t_RGB_B;
	uint32_t t_RGB_R;
	uint32_t t_Light;
	uint32_t t_Light_Q;//< ��ǰ
	uint32_t t_Light_Z;//< ���
	uint32_t t_Light_Y;//< ��ǰ
	uint32_t t_Light_H;//< �Һ�

	uint32_t t_cnt_RGB_G;
	uint32_t t_cnt_RGB_B;
	uint32_t t_cnt_RGB_R;
	uint32_t t_cnt_Light;
	uint32_t t_cnt_Light_Q;//< ��ǰ
	uint32_t t_cnt_Light_Z;//< ���
	uint32_t t_cnt_Light_Y;//< ��ǰ
	uint32_t t_cnt_Light_H;//< �Һ�

	uint32_t c_Light_Q:1;//< ��ǰ
	uint32_t c_Light_Z:1;//< ���
	uint32_t c_Light_Y:1;//< ��ǰ
	uint32_t c_Light_H:1;//< �Һ�
}LightTime;

extern struct Smooth_Control tagSmooth_control;  //�����ķ����Ҫ�õ���С���ٶ�ƽ������
extern float Move_X,Move_Y,Move_Z;   //С����������ٶ�
extern float Voltage;

/*----------------------���Ŀ��ƺ���--------------------------*/
void Ultrasonic1_task(void* pvParameters);
void Ultrasonic2_task(void* pvParameters);
void Balance_task(void *pvParameters);           //������
void Remote_Control(void);                       //��ģң�����������ݴ���
void Ros_Control(void);                          //��ģң�����������ݴ���
void Drive_Motor(float Vx,float Vy,float Vz);    //С���˶�ģ�ͣ��������������ٶ�
																							   //����4������ٶ�
void Set_MotorVelocity(int nMotorLB,int nMotorLF, int nMotorRF, int nMotorRB);    
void Get_Motor_Velocity(void);                   //��ȡ����ٶȷ���ֵ��r/min
void Smooth_control(float vx,float vy,float vz); //ȫ����С���ٶ�ƽ��
unsigned char Turn_Off(void);                    //���ݵ�ص���������
float RotateToSpeedVelocity(float nRotateSpeed);   //���ת��ת��Ϊ�������ٶȣ�m/s
int SpeedVelocityToRotate(float fltSpeed);       //�����ٶ�ת��Ϊ���ת��r/min
void Sensor_TX_Control(void);
void IrDA_Guide(void);
void IrDA_Send0(void);
void IrDA_Send1(void);
void IrDA_SendData(uint8_t data);
void Relay_Switch(void);

/*----------------------�������ܺ���--------------------------*/
uint32_t myabs(long int a);
float float_abs(float insert);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

#endif  

