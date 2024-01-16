#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H

#include "stdint.h"

#define CONTROL_DELAY		1000 // 换算成实际时间是10秒,为了消除后续读取陀螺仪的零点
#define RATE_1_HZ		  1      // 频率代表的是freeRTOS中每个任务的执行频率
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
// 枚举定义几种小车的类型
enum CarMode
{
	UNKNOW = -1,
	Mec_Car = 0,        // 麦克那母轮小车
	Omni_Car,           // 麦轮小车
	Akm_Car,            // 阿克曼
	Diff_Car,           // 差速车(圆形底盘)
	FourWheel_Car,      // 四驱车
	Tank_Car,           // 坦克车
	RC_Car,              // 竞赛小车
	Charger              // 充电桩
};


// 每个电机相关参数，有目标速度和反馈速度
struct Motor_parameter
{
	int nTarget_Velocity;        //单位r/min,为了便于做电机控制写入驱动器
	int nFeedback_Velocity;      //< rpm 单位(r/min)
	float fltTarget_velocity;    //做上下位机通信，传输的是轮子的速度m/s
	float fltFeedBack_Velocity;
};


// 枚举定义小车的控制方式
enum ENUM_CarControl_Mode
{
	CONTROL_MODE_UNKNOW = 0,    // 未知控制模式
	CONTROL_MODE_REMOTE = 1,    // 航模控制方式
	CONTROL_MODE_ROS = 2,    // 上位机控制模式
	CONTROL_MODE_UART = 3,    // 串口控制模式
	CONTROL_MODE_OTHER          // 其他控制模式
};
//航模设置结构体
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
//航模参数结构体指针
extern Remote_Control_struct* rc_ptr;
//电机参数结构体
typedef struct motor_struct
{
	uint16_t motor_direction;
	uint16_t motor_per;
	uint16_t reduction_ratio;
	uint16_t can_id;
	uint16_t can_baud_rate;
	uint16_t heartbeat_time;
}Motor_struct;

typedef struct  
{
  float WheelSpacing;      //轮距
  float AxleSpacing;       //轴距  
  float GearRatio;         //电机减速比
  float WheelDiameter;     //轮径
	float OmniTurnRadiaus; //全向轮旋转半径
}Robot_Parament_InitTypeDef;

typedef union __ROBOT_CONTROL
{
	struct
	{
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


//Car_Mode for Mec
//0:高配麦轮无轴承座SENIOR_MEC_NO  
//1:高配麦轮摆式悬挂SENIOR_MEC_BS  
//2:高配麦轮独立悬挂SENIOR_MEC_DL
//3:顶配麦轮摆式悬挂常规型TOP_MEC_BS_18
//4:顶配麦轮摆式悬挂重载型TOP_MEC_BS_47
//5:顶配麦轮独立悬挂常规型TOP_MEC_DL_18

//主动轮半宽度 注意是一半
#define MEC_wheelspacing         0.109
#define Akm_wheelspacing         0.155f
#define Diff_wheelSpacing        0.155f
#define Four_Mortor_wheelSpacing 0.705f    // 四驱差速车轮距
#define Tank_wheelSpacing     0.235f

//半轴距 注意是一半
#define MEC_axlespacing           0.085
#define Akm_axlespacing           0.155f
#define Diff_axlespacing          0.155f
#define Four_Mortor__axlespacing  0.660f   // 四驱差速车轴距
#define Tank_axlespacing       0.222f

//电机减速比
#define   HALL_30F    30
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47
#define   REDUCTION_RATE  28             // 四驱差速车所用电机减速器的减速比1：28



//麦轮轮胎直径
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f
 
//轮径全向轮直径系列
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//黑色轮胎、履带车轮径
#define	  Black_WheelDiameter   0.065
#define	  Tank_WheelDiameter 0.047

//全向轮小车旋转半径
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

#define FourWheer_Radiaus       0.165f     // 四驱差速车定义轮胎半径
extern float FourWheer_Perimeter;     // 四驱差速车轮胎周长
extern float FourWheer_Conversion;
extern float VelocityToRpmConversion;
extern float AngularVelocityConversion;

#define CONTROL_FREQUENCY 100
#define PI 3.1415f  //圆周率


extern Motor_struct* motorA_ptr;
extern Motor_struct* motorB_ptr;
extern Motor_struct* motorC_ptr;
extern Motor_struct* motorD_ptr;

/*--------------- 小车机械结构参数-----------------------*/
extern float Wheel_perimeter; //轮子周长（单位：米）
extern float Wheel_spacing;   //主动轮轮距 （单位：米）
extern float Axle_spacing;    //麦轮前后轴轴距
extern float Omni_turn_radiaus; //全向轮转弯半径


/*--------------- 初始化时的相关标志位-----------------------*/
extern enum CarMode g_emCarMode;                   //机器人选型标志位
extern unsigned char Flag_Stop;                    //机器人使能标志位，1代表关闭电机
extern enum ENUM_CarControl_Mode g_eControl_Mode;  //机器人的控制方式
extern unsigned char g_ucRemote_Flag;              //航模开启标志位
extern unsigned char g_ucRos_Flag;                 // ROS上位机进入标志位 

extern ROBOT_CONTROL robot_control;

extern struct Motor_parameter  MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

void Robot_Select(void);

void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float tyre_diameter);
void Soft_Reset(void);  // 手动软件复位
void Jump_To_BOOT(void);

extern uint16_t error_code;
extern uint16_t* getPDUData(void);

#endif
