#ifndef __user_can__h
#define __user_can__h

#include "can.h"
#include "n32g45x.h"

#define CAN_TASK_PRIO		20     //任务优先级
#define CAN_STK_SIZE 		128   //任务堆栈大小      


typedef struct {
    uint32_t id;
    uint8_t data[8];
} SDOMessage;

//宏定义从机的RPDO、TPDO帧ID
#define RPDO0_ID           	0x200
#define TPDO0_ID           	0x180
#define TPDO1_ID           	0x280
#define TPDO2_ID           	0x380

//宏定义从机的测试帧ID
#define SDO_M_ID           	0x600
#define SDO_S_ID			0x580
#define CHECKSUM_ID     	0x099

//宏定义从机的TPDO序号
#define RPDO0           	0
#define TPDO0           	0
#define TPDO1           	1
#define TPDO2           	2

extern SDOMessage receivedMsg,sendMsg,send1Msg,send2Msg;
extern int buff_cnt;

extern CanRxMessage M1T0Message, M2T0Message, M3T0Message, M4T0Message, M5T0Message, 
			 		M6T0Message, M7T0Message, M8T0Message, M9T0Message, M10T0Message,
			 		M1T1Message, M2T1Message, M3T1Message, M4T1Message, M5T1Message, 
			 		M6T1Message, M7T1Message, M8T1Message, M9T1Message, M10T1Message, 
			 		M1T2Message, M2T2Message, M3T2Message, M4T2Message, M5T2Message, 
			 		M6T2Message, M7T2Message, M8T2Message, M9T2Message, M10T2Message; 

extern bool M1T0Message_FLAG, M2T0Message_FLAG, M3T0Message_FLAG, M4T0Message_FLAG, M5T0Message_FLAG, 
	 		M6T0Message_FLAG, M7T0Message_FLAG, M8T0Message_FLAG, M9T0Message_FLAG, M10T0Message_FLAG,
	 		M1T1Message_FLAG, M2T1Message_FLAG, M3T1Message_FLAG, M4T1Message_FLAG, M5T1Message_FLAG, 
	 		M6T1Message_FLAG, M7T1Message_FLAG, M8T1Message_FLAG, M9T1Message_FLAG, M10T1Message_FLAG,
	 		M1T2Message_FLAG, M2T2Message_FLAG, M3T2Message_FLAG, M4T2Message_FLAG, M5T2Message_FLAG, 
	 		M6T2Message_FLAG, M7T2Message_FLAG, M8T2Message_FLAG, M9T2Message_FLAG, M10T2Message_FLAG;

extern uint32_t DiffposForOdom;

enum Offline_Time_Threshold
{
	Offline_Time_1s = 200,
	Offline_Time_2s = 400,	
	Offline_Time_3s = 600,
	Offline_Time_4s = 800,
	Offline_Time_5s = 1000,	
};

enum NMT_ORDER
{
	Start_Command 							= 0x01,	//启动
	Stop_Command 								= 0x02,	//停止
	Pre_Oper_Command 						= 0x80,	//预操作
	Reset_Application_Command 	= 0x81,	//复位节点应用层
	Reset_Command 							= 0x82,	//复位节点通讯
};

struct SdoFrame
{
	uint16_t ID;
	uint8_t mode;//<0:sdo,1:nmt
	uint8_t data[8];
	struct SdoFrame *next;
};
struct SdoFrame* SdoFrameCreate(void);

struct PointXYZ_6D
{
    float x;
    float y;
    float z;
    float roll;  // Roll角
    float pitch; // Pitch角
    float yaw;   // Yaw角
};

typedef union {
    struct PointXYZ_6D Waypoint_6D;
    float Waypoint_data[10];
} PointFrame_;

void Add_Sdo_Linked_List(uint8_t id, uint8_t Init_sdo[][8], int sdo_count);

void Can_Driver_Init(uint8_t baud);
void NMT_Control(const uint8_t Data0,const uint8_t ID); // NMT控制
void ZLAC8015_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl);// 中菱伺服
void ZLAC8015D_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl);// 中菱一拖二伺服
void WANZER_PDO_Config(uint8_t id, uint8_t mode);// 万泽伺服
void PLAN_PDO_Config(uint8_t id, uint8_t mode, uint8_t ah, uint8_t al, uint8_t dh, uint8_t dl);// 普蓝伺服
uint8_t IsSdoEmpty(void);
void Can_task(void* pvParameters);

void CAN_SDOSend(CAN_Module* CANx);
void CAN_PDOSend(uint32_t number, CAN_Module* CANx);

uint8_t Position_Servo_Init_ZLAC(uint8_t ID);                                 	 //位置模式初始配置 
uint8_t Velocity_Motor_Init_ZLAC(uint8_t ID);                                 	 //速度模式初始配置
uint8_t Torque_Motor_Init_ZLAC(uint8_t ID);                             
uint8_t Position_Servo_Init_WANZER(uint8_t ID);    
uint8_t Velocity_Motor_Init_WANZER(uint8_t ID);    
uint8_t Velocity_Motor_Init_PLAN(uint8_t ID);                                 	
uint8_t Position_Servo_Init_PLAN(uint8_t ID);     
void Driver_JT_Inv(uint8_t ID);						 //反转驱动电平			

#endif

