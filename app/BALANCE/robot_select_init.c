#include "robot_select_init.h"
#include "motor_data.h"
#include "485_address.h"
Robot_Parament_InitTypeDef  Robot_Parament;//初始化机器人参数结构体

/**************************************************************************
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	g_emCarMode = getPDUData()[car_model];   //设置为4驱差速
	if (g_emCarMode< Mec_Car || g_emCarMode>RC_Car)
	{
		g_emCarMode = FourWheel_Car;//< 默认为室外差速小车模型
	}
	getPDUData()[car_type] = g_emCarMode;
	g_ucRos_Flag = 0;
	g_ucRemote_Flag = 0;
	switch(g_emCarMode)
	{
		case FourWheel_Car: //四驱车 
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
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比  轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float tyre_diameter) // 
{
	Robot_Parament.WheelSpacing = wheelspacing;         //半轮距
	Robot_Parament.AxleSpacing = axlespacing;           //半轴距
  Robot_Parament.OmniTurnRadiaus = omni_turn_radiaus; //全向轮小车旋转半径	
  Robot_Parament.GearRatio = gearratio;               //电机减速比
  Robot_Parament.WheelDiameter = tyre_diameter;       //主动轮轮径
	
	Wheel_perimeter = Robot_Parament.WheelDiameter * PI;  //轮子周长
  Wheel_spacing = Robot_Parament.WheelSpacing;        //轮距 麦轮车为半轮距，左右轮距
	Axle_spacing = Robot_Parament.AxleSpacing;          //轴距 麦轮车为半轴距，前后轮距
	Omni_turn_radiaus = Robot_Parament.OmniTurnRadiaus; //全向轮小车旋转半径
}


