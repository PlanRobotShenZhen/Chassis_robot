#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H

#include "stdint.h"

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

#define ROBOT_Software_VERSION  0x02
#define ROBOT_Hardware_VERSION  0x02
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
	CONTROL_MODE_ROS = 2,    // ��λ������ģʽ
	CONTROL_MODE_UART = 3,    // ���ڿ���ģʽ
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
extern Remote_Control_struct* rc_ptr;
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

typedef struct  
{
  float WheelSpacing;      //�־�
  float AxleSpacing;       //���  
  float GearRatio;         //������ٱ�
  float WheelDiameter;     //�־�
	float OmniTurnRadiaus; //ȫ������ת�뾶
}Robot_Parament_InitTypeDef;



//Car_Mode for Mec
//0:���������������SENIOR_MEC_NO  
//1:�������ְ�ʽ����SENIOR_MEC_BS  
//2:�������ֶ�������SENIOR_MEC_DL
//3:�������ְ�ʽ���ҳ�����TOP_MEC_BS_18
//4:�������ְ�ʽ����������TOP_MEC_BS_47
//5:�������ֶ������ҳ�����TOP_MEC_DL_18

//�����ְ��� ע����һ��
#define MEC_wheelspacing         0.109
#define Akm_wheelspacing         0.155f
#define Diff_wheelSpacing        0.155f
#define Four_Mortor_wheelSpacing 0.705f    // �������ٳ��־�
#define Tank_wheelSpacing     0.235f

//����� ע����һ��
#define MEC_axlespacing           0.085
#define Akm_axlespacing           0.155f
#define Diff_axlespacing          0.155f
#define Four_Mortor__axlespacing  0.660f   // �������ٳ����
#define Tank_axlespacing       0.222f

//������ٱ�
#define   HALL_30F    30
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47
#define   REDUCTION_RATE  28             // �������ٳ����õ���������ļ��ٱ�1��28



//������ֱ̥��
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 
//�־�ȫ����ֱ��ϵ��
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//��ɫ��̥���Ĵ����־�
#define	  Black_WheelDiameter   0.065
#define	  Tank_WheelDiameter 0.047

//ȫ����С����ת�뾶
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

#define FourWheer_Radiaus       0.165f     // �������ٳ�������̥�뾶
extern float FourWheer_Perimeter;     // �������ٳ���̥�ܳ�


#define CONTROL_FREQUENCY 100
#define PI 3.1415f  //Բ����


extern Motor_struct* motorA_ptr;
extern Motor_struct* motorB_ptr;
extern Motor_struct* motorC_ptr;
extern Motor_struct* motorD_ptr;

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

void Robot_Select(void);

void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float tyre_diameter);
void Soft_Reset(void);  // �ֶ������λ
extern uint16_t* getPDUData(void);

#endif
