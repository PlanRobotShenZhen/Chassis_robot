#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H

#include "stdint.h"


// 枚举定义几种小车的类型

enum enum_CarType
{
	Unknown= -1,
	Mec_Car = 0,        // 麦轮小车
	Omni_Car,           // 万向轮小车
	Akm_Car,            // 阿克曼
	Diff_Car,           // 差速车(圆形底盘)
	FourWheel_Car,      // 室外差速四驱车
	TwoWheel_Car,      // 室内差速二驱车
	Tank_Car,           // 坦克车
	RC_Car,              // 竞赛小车
	Charger              // 充电桩
};
extern enum enum_CarType CarType;

// 每个电机相关参数，有目标速度和反馈速度
struct Motor_parameter
{
	int nTarget_Velocity;        //单位r/min,为了便于做电机控制写入驱动器
	int nFeedback_Velocity;      //< rpm 单位(r/min)
	float fltTarget_velocity;    //做上下位机通信，传输的是轮子的速度m/s
	float fltFeedBack_Velocity;
};





typedef struct{
  uint16_t  car_type;
  uint16_t  car_product_number;
  uint16_t  car_version;
  uint16_t  car_length;
  uint16_t  car_width;
  uint16_t  car_height;
  uint16_t  car_wheelbase;
  uint16_t  car_tread;
  uint16_t  car_ground_clearance;
  uint16_t  wheel_radius;
  uint16_t  gross_max;
  uint16_t  rated_load;
  uint16_t  motor_number;
  uint16_t  driving_method;
}struct_RobotBasePara;

typedef union __ROBOT_CONTROL
{
	struct{
		uint8_t motor_en:1;//< 电机使能控制位
		uint8_t light_ctrl_en:1;//< 车灯使能控制位
		uint8_t res1 : 1;//< 预留位
		uint8_t res2 : 1;//< 预留位
		uint8_t res3 : 1;//< 预留位
		uint8_t res4 : 1;//< 预留位
		uint8_t res5 : 1;//< 预留位
		uint8_t res6 : 1;//< 预留位
	}bit;
	uint8_t ctrl;
}ROBOT_CONTROL;
extern ROBOT_CONTROL robot_control;

#define PI 3.1415926f  //圆周率



//void Robot_Select(void);

void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float tyre_diameter);
void Soft_Reset(void);  // 手动软件复位
void Jump_To_BOOT(void);

extern uint16_t* getPDUData(void);

#endif
