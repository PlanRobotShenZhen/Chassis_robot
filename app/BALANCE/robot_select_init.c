#include "robot_select_init.h"
#include "motor_data.h"
#include "485_address.h"
Robot_Parament_InitTypeDef  Robot_Parament;//��ʼ�������˲����ṹ��

/**************************************************************************
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	g_emCarMode = getPDUData()[car_model];   //����Ϊ4������
	if (g_emCarMode< Mec_Car || g_emCarMode>RC_Car)
	{
		g_emCarMode = FourWheel_Car;//< Ĭ��Ϊ�������С��ģ��
	}
	getPDUData()[car_type] = g_emCarMode;
	g_ucRos_Flag = 0;
	g_ucRemote_Flag = 0;
	switch(g_emCarMode)
	{
		case FourWheel_Car: //������ 
			Motor_Number = 4;
			Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0, REDUCTION_RATE, Black_WheelDiameter);   
		break;
		case Akm_Car:
			Motor_Number = 3;
			break;
		case Diff_Car:
			Motor_Number = 2;
			Slave_Number = 1;
			break;
		case RC_Car:
			Motor_Number = 1;
			break;
		default:
			break;
	}
}


/**************************************************************************
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ�  ��ֱ̥��
����  ֵ����
**************************************************************************/
void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float tyre_diameter) // 
{
	Robot_Parament.WheelSpacing = wheelspacing;         //���־�
	Robot_Parament.AxleSpacing = axlespacing;           //�����
  Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus; //ȫ����С����ת�뾶	
  Robot_Parament.GearRatio = gearratio;               //������ٱ�
  Robot_Parament.WheelDiameter = tyre_diameter;       //�������־�
	
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;  //�����ܳ�
  Wheel_spacing = Robot_Parament.WheelSpacing;        //�־� ���ֳ�Ϊ���־࣬�����־�
	Axle_spacing = Robot_Parament.AxleSpacing;          //��� ���ֳ�Ϊ����࣬ǰ���־�
	Omni_turn_radiaus = Robot_Parament.OmniTurnRadiaus; //ȫ����С����ת�뾶
}


