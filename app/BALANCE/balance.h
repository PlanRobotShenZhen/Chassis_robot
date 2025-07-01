#ifndef __BALANCE_H
#define __BALANCE_H			

#include "stdint.h"
#include"stdbool.h"

#define BALANCE_TASK_PRIO 4	 //任务优先级
#define BALANCE_STK_SIZE 512 //任务堆栈大小

#define VELOCITY_MULTI 3 //速度值倍率

//角速度增益参数
#define AKM_RAW_FACTOR 1.277f

#define EPSILON 1e-6  // 适当的极小值
#define CORRECTION_FACTOR 1.35f  // 调整系数
#define ENCODER_LINES 10000  //一圈脉冲数默认为10000
#define MYHALF 0.5f  //一半(0.5)
#define RADtoANG 180.0f / PI	//弧度->角度
#define ANGtoRAD PI / 180.0f	//角度->弧度
#define DEGREE_MAX 300.0f 	//前轮最大转角


#define MAGNIFIC_10x_DOWN 		0.1f		//缩小10倍
#define MAGNIFIC_100x_DOWN 		0.01f		//缩小100倍
#define MAGNIFIC_1000x_DOWN 	0.001f		//缩小1000倍
#define MAGNIFIC_10000x_DOWN 	0.0001f		//缩小10000倍
#define MAGNIFIC_10x_UP 		10.0f		//增大10倍
#define MAGNIFIC_100x_UP 		100.0f		//增大100倍
#define MAGNIFIC_1000x_UP 		1000.0f		//增大1000倍
#define MAGNIFIC_10000x_UP 		10000.0f	//增大10000倍
#define MINTOSEC 				60.0f		//分钟->秒钟

//SWB速度模式下的力矩挡位(VRA控制)系数
#define SWB_LOW_GEAR 			20			//力矩低档位系数
#define SWB_MIDDLE_GEAR 		70			//力矩中档位系数
#define SWB_HIGH_GEAR 			130			//力矩高档位系数 

#define CHANNEL_VALUE_ERROR 	10			//手柄通道值允许误差

#define TORQUE_COEFFICIENT_MAX 	3			//最大力矩档
#define TORQUE_COEFFICIENT_BASE 2			//中等力矩档
#define TORQUE_COEFFICIENT_MIN 	1			//最小力矩档

extern float Move_X,Move_Y,Move_Z;   	//小车各个轴的速度
extern float Voltage;
extern float Z_Radian_Max; 				//弧度最大值
extern bool ROS_JT_Flag;

/*----------------------各小车速度宏定义--------------------------*/
#define MAXDEGREE 300 		//前轮最大转角
//Akm_Car
#define AKMCAR_MAXLINEARVELOCITY 		1728									//最大线速度
#define AKMCAR_MAXYAWVELOCITY 			1330									//最大角速度
#define AKMCAR_SPEED_LOW 				AKMCAR_MAXLINEARVELOCITY / 3			//低速档线速度系数
#define AKMCAR_SPEED_MIDDLE 			AKMCAR_MAXLINEARVELOCITY / 3 * 2		//中速档线速度系数
#define AKMCAR_SPEED_HIGH 				AKMCAR_MAXLINEARVELOCITY				//高速档线速度系数
#define AKMCAR_YAW_LOW 					AKMCAR_MAXYAWVELOCITY / 3				//低速档角速度系数
#define AKMCAR_YAW_MIDDLE 				AKMCAR_MAXYAWVELOCITY / 3 * 2			//中速档角速度系数
#define AKMCAR_YAW_HIGH 				AKMCAR_MAXYAWVELOCITY					//高速档角速度系数
//Diff_Car		
#define DIFFCAR_MAXLINEARVELOCITY 		800										//最大线速度
#define DIFFCAR_MAXYAWVELOCITY 			1500									//最大角速度
#define DIFFCAR_SPEED_LOW 				DIFFCAR_MAXLINEARVELOCITY / 3			//低速档线速度系数
#define DIFFCAR_SPEED_MIDDLE 			DIFFCAR_MAXLINEARVELOCITY / 3 * 2		//中速档线速度系数
#define DIFFCAR_SPEED_HIGH 				DIFFCAR_MAXLINEARVELOCITY				//高速档线速度系数
#define DIFFCAR_YAW_LOW 				DIFFCAR_MAXYAWVELOCITY / 15 * 3			//低速档角速度系数
#define DIFFCAR_YAW_MIDDLE 				DIFFCAR_MAXYAWVELOCITY / 15 * 5 		//中速档角速度系数
#define DIFFCAR_YAW_HIGH 				DIFFCAR_MAXYAWVELOCITY / 15 * 8			//高速档角速度系数
//FourWheel_Car		
#define FOURWHEELCAR_MAXLINEARVELOCITY 	1500									//最大线速度
#define FOURWHEELCAR_MAXYAWVELOCITY 	3000									//最大角速度
#define FOURWHEELCAR_SPEED_LOW 			FOURWHEELCAR_MAXLINEARVELOCITY / 3		//低速档线速度系数
#define FOURWHEELCAR_SPEED_MIDDLE 		FOURWHEELCAR_MAXLINEARVELOCITY / 3 * 2	//中速档线速度系数
#define FOURWHEELCAR_SPEED_HIGH 		FOURWHEELCAR_MAXLINEARVELOCITY			//高速档线速度系数
#define FOURWHEELCAR_YAW_LOW 			FOURWHEELCAR_MAXYAWVELOCITY / 3			//低速档角速度系数
#define FOURWHEELCAR_YAW_MIDDLE 		FOURWHEELCAR_MAXYAWVELOCITY / 3 * 2		//中速档角速度系数
#define FOURWHEELCAR_YAW_HIGH 			FOURWHEELCAR_MAXYAWVELOCITY				//高速档角速度系数
//TwoWheel_Car	
#define TWOWHEELCAR_MAXLINEARVELOCITY 	1000									//最大线速度
#define TWOWHEELCAR_MAXYAWVELOCITY 		1500									//最大角速度
#define TWOWHEELCAR_SPEED_LOW 			TWOWHEELCAR_MAXLINEARVELOCITY / 3		//低速档线速度系数
#define TWOWHEELCAR_SPEED_MIDDLE 		TWOWHEELCAR_MAXLINEARVELOCITY / 3 * 2	//中速档线速度系数
#define TWOWHEELCAR_SPEED_HIGH 			TWOWHEELCAR_MAXLINEARVELOCITY			//高速档线速度系数
#define TWOWHEELCAR_YAW_LOW 			TWOWHEELCAR_MAXYAWVELOCITY / 3			//低速档角速度系数
#define TWOWHEELCAR_YAW_MIDDLE 			TWOWHEELCAR_MAXYAWVELOCITY / 3 * 2		//中速档角速度系数
#define TWOWHEELCAR_YAW_HIGH 			TWOWHEELCAR_MAXYAWVELOCITY				//高速档角速度系数
//Tank_Car	
#define TANKCAR_MAXLINEARVELOCITY 		1000									//最大线速度
#define TANKCAR_MAXYAWVELOCITY 			1500									//最大角速度
#define TANKCAR_SPEED_LOW 				TANKCAR_MAXLINEARVELOCITY / 3			//低速档线速度系数
#define TANKCAR_SPEED_MIDDLE 			TANKCAR_MAXLINEARVELOCITY / 3 * 2		//中速档线速度系数
#define TANKCAR_SPEED_HIGH 				TANKCAR_MAXLINEARVELOCITY				//高速档线速度系数
#define TANKCAR_YAW_LOW 				TANKCAR_MAXYAWVELOCITY / 3				//低速档角速度系数
#define TANKCAR_YAW_MIDDLE 				TANKCAR_MAXYAWVELOCITY / 3 * 2			//中速档角速度系数
#define TANKCAR_YAW_HIGH 				TANKCAR_MAXYAWVELOCITY					//高速档角速度系数

/*----------------------核心控制函数--------------------------*/
void Ultrasonic1_task(void* pvParameters);
void Balance_task(void *pvParameters);           //任务函数
unsigned char Turn_Off(void);                    //根据电池电量来操作
void IrDA_TX_Control(void);
void IrDA_Guide(void);
void IrDA_Send0(void);
void IrDA_Send1(void);
void IrDA_SendData(uint8_t data);
void IrDA_RX_Decode(void);
void Relay_Switch(void);								// 继电器开关控制函数，用于控制继电器的打开和关闭
void ChargerBalanceInit(void);
void SetReal_Velocity(void);                         //设置电机运行的速度       
void ClassificationOfMotorMotionModes(uint16_t sport_mode);	   
void ServoPulse_Enable(void);             
void BatteryInformation(void);
void SPI1_ReadWriteByte(void);
void PowerControl(void);
/*----------------------辅助功能函数--------------------------*/
uint32_t myabs(long int a);
float float_abs(float insert);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
short LinearVelocityGet(void);
short VirtuallyLinearVelocityGet(void);
short AngularVelocityGet(short linearValue);
short VirtuallyAngularVelocityGet(short linearValue);
void Kinematic_Analysis(short Vx, float Vy, short Vz);
#endif  

