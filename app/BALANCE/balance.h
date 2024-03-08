#ifndef __BALANCE_H
#define __BALANCE_H			

#include "stdint.h"

#define BALANCE_TASK_PRIO		11     //任务优先级
#define BALANCE_STK_SIZE 		256   //任务堆栈大小
#define DIRECTOR_BASE       784   //Z轴角速度的基准

//全向轮机器人数学模型参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)  

//充电桩相关参数
#define LED_LEFT 			Light_Q
#define LED_RIGHT 			Light_H
#define MCU_INF_TX 			exio_output.bit.RGB_G
#define MCU_RELAY2 			exio_output.bit.RGB_B
#define IrDA_TX 			MCU_INF_TX
//风扇相关参数
#define MCU_FAN1			exio_output.bit.Light_Y
#define MCU_FAN2			exio_output.bit.Light_Z
#define FAN1				MCU_FAN1
#define FAN2				MCU_FAN2
//充电电极短路检测
#define MCU_CH_DET			exio_input.bit.CS2_Ttig
//存放平滑控制后的数据
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
	uint32_t t_Light_Q;//< 左前
	uint32_t t_Light_Z;//< 左后
	uint32_t t_Light_Y;//< 右前
	uint32_t t_Light_H;//< 右后

	uint32_t t_cnt_RGB_G;
	uint32_t t_cnt_RGB_B;
	uint32_t t_cnt_RGB_R;
	uint32_t t_cnt_Light;
	uint32_t t_cnt_Light_Q;//< 左前
	uint32_t t_cnt_Light_Z;//< 左后
	uint32_t t_cnt_Light_Y;//< 右前
	uint32_t t_cnt_Light_H;//< 右后

	uint32_t c_Light_Q:1;//< 左前
	uint32_t c_Light_Z:1;//< 左后
	uint32_t c_Light_Y:1;//< 右前
	uint32_t c_Light_H:1;//< 右后
}LightTime;

extern struct Smooth_Control tagSmooth_control;  //麦克纳姆轮需要用到的小车速度平滑处理
extern float Move_X,Move_Y,Move_Z;   //小车各个轴的速度
extern float Voltage;

/*----------------------核心控制函数--------------------------*/
void Ultrasonic1_task(void* pvParameters);
void Ultrasonic2_task(void* pvParameters);
void Balance_task(void *pvParameters);           //任务函数
void Remote_Control(void);                       //航模遥控器接收数据处理
void Ros_Control(void);                          //航模遥控器接收数据处理
void Drive_Motor(float Vx,float Vy,float Vz);    //小车运动模型，解算出各个电机速度
																							   //设置4个电机速度
void Set_MotorVelocity(int nMotorLB,int nMotorLF, int nMotorRF, int nMotorRB);    
void Get_Motor_Velocity(void);                   //获取电机速度反馈值，r/min
void Smooth_control(float vx,float vy,float vz); //全向轮小车速度平滑
unsigned char Turn_Off(void);                    //根据电池电量来操作
float RotateToSpeedVelocity(float nRotateSpeed);   //电机转速转换为轮子线速度，m/s
int SpeedVelocityToRotate(float fltSpeed);       //轮子速度转换为电机转速r/min
void Sensor_TX_Control(void);
void IrDA_Guide(void);
void IrDA_Send0(void);
void IrDA_Send1(void);
void IrDA_SendData(uint8_t data);
void Relay_Switch(void);

/*----------------------辅助功能函数--------------------------*/
uint32_t myabs(long int a);
float float_abs(float insert);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);

#endif  

