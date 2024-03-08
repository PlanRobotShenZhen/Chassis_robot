#include "balance.h"
#include "user_can.h"
#include "motor_data.h"
#include "485_address.h"
#include "robot_select_init.h" 
#include "usartx.h"
#include "led.h"
#include "port.h"
#include "rc_car.h"
#include <math.h>
#include "Charger.h"
struct Smooth_Control tagSmooth_control;
LightTime light_time;
float Move_X = 0,Move_Y = 0,Move_Z = 0;
static uint16_t* pdu;
uint16_t error_code = ERROR_NONE;
float FourWheer_Perimeter;
float FourWheer_Conversion;
float VelocityToRpmConversion;//速度换算电机转速
float AngularVelocityConversion;//角速度换算
int g_nVol_get_Flag = 0;          //
float g_fltProprity_Voltage=1;  //
float Voltage = 0.0f;

uint8_t ultrasonic_t1tig = 1;
uint8_t ultrasonic_t2tig = 1;
uint32_t ultrasonic_t1tig_time = 0;
uint32_t ultrasonic_t2tig_time = 0;
int ultrasonic_t1tig_heartbeat = 0;
int ultrasonic_t2tig_heartbeat = 0;

uint8_t f1_cnt = 0;
uint32_t ultrasonic1_filtering[4];
uint8_t f2_cnt = 0;
uint32_t ultrasonic2_filtering[4];

uint8_t SPI_Master_Rx_Buffer;
int SPI_heartbeat = 0;
uint8_t SPI_ReadWriteCycle = 0;
EXIO_INPUT exio_input;
EXIO_OUTPUT exio_output;

/*红外通讯发送部分参数*/
uint8_t IrDA_SendState = 0;		//0：关闭红外传感器；1：红外对接；2：红外通讯；
uint8_t SendCout = 0;			//发送计次
uint8_t SendGuide_Flag = 0;		//引导位状态
int Send_i = 3;					//发送数据指针，从高位开始发送
int BitFree = 1;				//发送通道空闲
uint8_t SendData = 0x01;		//要发送的数据
/*红外通讯接收部分参数*/
uint8_t ReceiveData;			//接收到的数据
int LimitSwitch_OK = 0;		//限位开关，断开0，闭合1；

struct {
	unsigned char SW : 2;//《 急停开关
	unsigned char Rising : 1;
	unsigned char Descending : 1;

	unsigned char estop_soft : 1;//< 软急停
	unsigned char estop_soft_old : 1;//< 软急停
}emergency_stop;

static uint8_t battery_send_frame_num = 0;
/**************************************************************************
函数功能：对接收到数据进行处理
入口参数：X和Y Z轴方向的运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
	float Wheel_spacing_diff = (float)pdu[wheel_distance]/10000;
	float Axle_spacing_diff = (float)pdu[axles_distance]/10000;
	union {
		float v;
		uint16_t ud[2];
		uint8_t u8d[8];
	}tmp;
	int i;
	//四驱车 ---- 当前所用到的车
	if(g_emCarMode == FourWheel_Car) 
	{	//用的时候motora和motorb要设置的为负数		
		float tmp_value = Vz * (Wheel_spacing +  Axle_spacing) / 2.0f;
		MOTOR_A.fltTarget_velocity = Vx + tmp_value;
		MOTOR_B.fltTarget_velocity = MOTOR_A.fltTarget_velocity; //计算出左前轮的目标速度
		MOTOR_C.fltTarget_velocity = Vx - tmp_value;
		MOTOR_D.fltTarget_velocity = MOTOR_C.fltTarget_velocity;

		int tva = SpeedVelocityToRotate(MOTOR_A.fltTarget_velocity); //转换为r/min
		//if (pdu[motor1_model] == SERVO_WANZE || pdu[motor1_model] == SERVO_PLAN)tva *= 500;
		int tvb = tva;
		int tvc = SpeedVelocityToRotate(MOTOR_C.fltTarget_velocity);
		//if (pdu[motor1_model] == SERVO_WANZE || pdu[motor1_model] == SERVO_PLAN)tvc *=500;
		int tvd = tvc;

		int diff = tva - MOTOR_A.nTarget_Velocity;
		int acc = 500;
		//if (pdu[motor1_model] == SERVO_WANZE || pdu[motor1_model] == SERVO_PLAN)
		//{
		//	if (exio_input.bit.X0 || emergency_stop.estop_soft)
		//	{//< 急停
		//		acc = 50000;
		//	}
		//	else acc = pdu[robot_acceleration];
		//}
		if (exio_input.bit.X0 || emergency_stop.estop_soft)
		{//< 急停
			acc = 50000;
		}
		else acc = pdu[robot_acceleration];
		if (diff > acc)tva = MOTOR_A.nTarget_Velocity + acc;
		else if (diff < -acc)tva = MOTOR_A.nTarget_Velocity - acc;

		diff = tvb - MOTOR_B.nTarget_Velocity;
		if (diff > acc)tvb = MOTOR_B.nTarget_Velocity + acc;
		else if (diff < -acc)tvb = MOTOR_B.nTarget_Velocity - acc;

		diff = tvc - MOTOR_C.nTarget_Velocity;
		if (diff > acc)tvc = MOTOR_C.nTarget_Velocity + acc;
		else if (diff < -acc)tvc = MOTOR_C.nTarget_Velocity - acc;

		diff = tvd - MOTOR_D.nTarget_Velocity;
		if (diff > acc)tvd = MOTOR_D.nTarget_Velocity + acc;
		else if (diff < -acc)tvd = MOTOR_D.nTarget_Velocity - acc;


		MOTOR_A.nTarget_Velocity = tva;
		MOTOR_B.nTarget_Velocity = tvb;
		MOTOR_C.nTarget_Velocity = tvc;
		MOTOR_D.nTarget_Velocity = tvd;
		
	}
	else if (g_emCarMode == RC_Car)
	{//< 竞赛小车

	}
}



/**************************************************************************
函数功能：通过航模遥控对机器人进行遥控,只需要x轴速度与z轴角速度
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control()
{
	//对航模解析到的数值转换成线速度和角速度
	//Move_X = ((float)g_nVelocity * 2 * PI * pdu[61] / 100) / 60 / pdu[62];    //单位为m/s,这里pdu[61]要除以10000，pdu[62]要除以100，则只需除以100
	Move_X = (float)g_nVelocity * FourWheer_Conversion;    //单位为m/s
	Move_Y = 0;
	Move_Z = ((float)g_nDirector) / AngularVelocityConversion;

}	



/**************************************************************************
函数功能：通过上位机对机器人进行遥控,只需要x轴速度与z轴角速度
入口参数：无
返回  值：无
**************************************************************************/
void Ros_Control()
{
	Move_X = g_fltRecv_Vel_X;
	Move_Y = g_fltRecv_Vel_Y;
	Move_Z = -g_fltRecv_Vel_Z;    // 需要反向Z轴数据，调试中发现的
}	


/**************************************************************************
函数功能：获取4个电机速度。根据can中断，获取得到4个电机的速度，
					存放在对应的Motor结构体中。r/min
入口参数：无
返 回 值：无
**************************************************************************/
void Get_Motor_Velocity()
{
	//这里采集回来的是驱动器反馈回来的数据，要经过相应转换才行
	if (pdu[car_model] == RC_Car)
	{
		pdu[car_setting_lin_speed] = (int16_t)(Move_X * 1000);
		pdu[car_setting_ang_speed] = (int16_t)(Move_Z * 1000);
	}
	else if (pdu[car_model] == FourWheel_Car)
	{//< 室外差速
		uint32_t vel_;
		pdu[car_setting_lin_speed] = (int16_t)(-Move_X * 1000);
		pdu[car_setting_ang_speed] = (int16_t)(-Move_Z * 1000);
		float nMotor_A = mtd[0].d.current_velocity;
		float nMotor_B = mtd[1].d.current_velocity;
		float nMotor_C = mtd[2].d.current_velocity;
		float nMotor_D = mtd[3].d.current_velocity;
		if (pdu[motor1_model] == SERVO_WANZE||pdu[motor1_model] == SERVO_PLAN)nMotor_A = nMotor_A * 60 / 10000;//< 一圈10000脉冲数
		if (pdu[motor2_model] == SERVO_WANZE||pdu[motor2_model] == SERVO_PLAN)nMotor_B = nMotor_B * 60 / 10000;//< 一圈10000脉冲数
		if (pdu[motor3_model] == SERVO_WANZE||pdu[motor3_model] == SERVO_PLAN)nMotor_C = nMotor_C * 60 / 10000;//< 一圈10000脉冲数
		if (pdu[motor4_model] == SERVO_WANZE||pdu[motor4_model] == SERVO_PLAN)nMotor_D = nMotor_D * 60 / 10000;//< 一圈10000脉冲数
		if (pdu[motor1_model] == SERVO_WANZE)mtd[0].d.current_rpm=(int16_t)nMotor_A;
		if (pdu[motor2_model] == SERVO_WANZE)mtd[1].d.current_rpm=(int16_t)nMotor_B;
		if (pdu[motor3_model] == SERVO_WANZE)mtd[2].d.current_rpm=(int16_t)nMotor_C;
		if (pdu[motor4_model] == SERVO_WANZE)mtd[3].d.current_rpm=(int16_t)nMotor_D;

		MOTOR_A.nFeedback_Velocity = (int)nMotor_A;
		MOTOR_B.nFeedback_Velocity = (int)nMotor_B;
		MOTOR_C.nFeedback_Velocity = (int)nMotor_C;
		MOTOR_D.nFeedback_Velocity = (int)nMotor_D;	
		MOTOR_A.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_A);
		MOTOR_B.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_B);
		MOTOR_C.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_C);
		MOTOR_D.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_D);
		float fv = (MOTOR_A.fltFeedBack_Velocity-MOTOR_C.fltFeedBack_Velocity)/2;
		int16_t iv = (int16_t)(fv * 1000);
		pdu[car_feedback_lin_speed] = iv;
		fv = (MOTOR_A.fltFeedBack_Velocity + MOTOR_C.fltFeedBack_Velocity) / 2 / (pdu[wheel_distance] + pdu[axles_distance]) / 10000;
		iv = (int16_t)(fv * 1000);
		pdu[car_feedback_ang_speed] = iv;

	}
}


/**************************************************************************
函数功能：将电机的转速，r/min转换成轮子线速度 m/s，
					存放在速度结构体中。
					1 r/min = （2Π * R） / 60  m/s
入口参数：nRotateSpeed代表转速，r/min
返 回 值：轮子线速度 m/s，
**************************************************************************/
float RotateToSpeedVelocity(float nRotateSpeed)
{
	//float fltReturn = 0.0f;
	//v=n*2*pi*r/60
	//fltReturn = nRotateSpeed * 2 * PI * FourWheer_Radiaus/ 60;  //转换为轮胎速度
	// 2 * PI * FourWheer_Radiaus/ 60 = 0.0172787593
	//fltReturn = nRotateSpeed * 0.0172787593;  //转换为轮胎速度
	return nRotateSpeed * FourWheer_Conversion;  //转换为轮胎速度;	
}


/**************************************************************************
函数功能：将轮子的速度，m/s转换成电机的转速 r/min，
					存放在速度结构体中。
					1 m/s = 60 /（2 * PI * R）
入口参数：fltSpeed代表轮子转速，m/s
返 回 值：代表电机转速
**************************************************************************/
int SpeedVelocityToRotate(float fltSpeed)
{
	int nReturn = 0;
	//v=n*2*pi*r/60
	// nReturn = (60 * fltSpeed / FourWheer_Perimeter) * REDUCTION_RATE;
	nReturn = fltSpeed * VelocityToRpmConversion;
	return nReturn;
}



/**************************************************************************
函数功能：将机器人的目标速度做平滑控制处理，只有麦克纳姆轮小车需要
入口参数：机器人三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	(vx>0) 			tagSmooth_control.Vx += step;
	else if(vx<0)		tagSmooth_control.Vx -= step;
	else if(vx==0)	tagSmooth_control.Vx = tagSmooth_control.Vx * 0.9;
	
	if	(vy>0) 			tagSmooth_control.Vy += step;
	else if(vy<0)		tagSmooth_control.Vy -= step;
	else if(vy==0)	tagSmooth_control.Vy = tagSmooth_control.Vy * 0.9;
	
	if	(vz>0) 			tagSmooth_control.Vz += step;
	else if(vz<0)		tagSmooth_control.Vz -= step;
	else if(vz==0)	tagSmooth_control.Vz = tagSmooth_control.Vz*0.9;
	
	tagSmooth_control.Vx = target_limit_float(tagSmooth_control.Vx,-float_abs(vx),float_abs(vx));
	tagSmooth_control.Vy = target_limit_float(tagSmooth_control.Vy,-float_abs(vy),float_abs(vy));
	tagSmooth_control.Vz = target_limit_float(tagSmooth_control.Vz,-float_abs(vz),float_abs(vz));
}




/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
unsigned char Turn_Off( )
{
	unsigned char ucTemp;
	if((g_nVol_get_Flag && g_fltProprity_Voltage < 0.2) || Flag_Stop == 1)//电池电压过低关闭电机
	{	 
		//设置电机目标速度为0，且失能电机
		ucTemp = 1;      
		Set_MotorVelocity(0, 0, 0, 0);   //设置目标电机的速度
	}
	else if(g_nVol_get_Flag && g_fltProprity_Voltage >= 0.2)
	{
		ucTemp = 0;
	}
	return ucTemp;			
}


/**************************************************************************
函数功能：设置4个电机速度
入口参数：参数分别代表的是左后、左前、右前、右后方对应的can从机id和电机速度
返 回 值：无
**************************************************************************/
void Set_MotorVelocity(int nMotorLB,int nMotorLF, int nMotorRF, int nMotorRB)
{//Is_Offline();target_velocity
	mrd[0].d.target_velocity = mrd[0].d.online ? nMotorLB : 0;
	mrd[1].d.target_velocity = mrd[1].d.online ? nMotorLF : 0;
	mrd[2].d.target_velocity = mrd[2].d.online ? nMotorRB : 0;
	mrd[3].d.target_velocity = mrd[3].d.online ? nMotorRF : 0;
	if(pdu[motor1_direction]==1)mrd[0].d.target_velocity = -mrd[0].d.target_velocity;
	if(pdu[motor2_direction]==1)mrd[1].d.target_velocity = -mrd[1].d.target_velocity;
	if(pdu[motor3_direction]==1)mrd[2].d.target_velocity = -mrd[2].d.target_velocity;
	if(pdu[motor4_direction]==1)mrd[3].d.target_velocity = -mrd[3].d.target_velocity;
}

/*---------------------------一些功能函数--------------------------------*/
/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{ 		   
	  uint32_t temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}



/**************************************************************************
函数功能：浮点型数据取绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


/**************************************************************************
函数功能：限幅函数，设定高低阈值
入口参数：幅值
返回  值：
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

void PowerControl(void)
{
	static union {
		struct {
			uint16_t jdq1 : 1;//< 左电机电源 JDQ1_EN
			uint16_t jdq2 : 1;//< 右电机电源 JDQ2_EN
			uint16_t p12v : 1;//< 12V电源 YL_7
			uint16_t p19v : 1;//< 19V电源 YL_6

		}bit;
		uint16_t pc;
	}power_ctrl;
	if (pdu[power_control] != power_ctrl.pc)
	{
		power_ctrl.pc = pdu[power_control];
		if (power_ctrl.bit.jdq1==1)
		{
			GPIO_ResetBits(JDQ_PORT, JDQ1_PIN);
		}
		else
		{
			GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
		}
		if (power_ctrl.bit.jdq2 == 1)
		{
			GPIO_ResetBits(JDQ_PORT, JDQ2_PIN);
		}
		else
		{
			GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
		}
		if (power_ctrl.bit.p12v == 1)
		{
			GPIO_ResetBits(YL_7_GPIO, YL_7_Pin);//< 
		}
		else
		{
			GPIO_SetBits(YL_7_GPIO, YL_7_Pin);//< 
		}
		if (power_ctrl.bit.p19v == 1)
		{
			GPIO_ResetBits(YL_6_GPIO, YL_6_Pin);//< 
		}
		else
		{
			GPIO_SetBits(YL_6_GPIO, YL_6_Pin);//< 
		}
	}
}

void BatteryInformation()
{
	static int bt = 0;
	static int bt_times = 100;
	int i = 0; //起始索引
	float MOVE_XorZ = 0;
	float temp;
	if (uart4_recv_flag)
	{
		uart4_recv_flag = 0;
		switch (pdu[battery_manufacturer])
		{
		case 1://< 深圳市锂神科技有限公司
			//小车线速度和角速度比较
			if (fabsf(Move_X) > fabsf(Move_Z))
			{
				MOVE_XorZ = Move_X;
				temp = ((int16_t)pdu[car_max_lin_speed]) * 0.001;
			} //起始索引
			else
			{
				MOVE_XorZ = Move_Z;
				temp = ((int16_t)pdu[car_max_ang_speed]) * 0.001;
			}
			if (MOVE_XorZ < 0)
			{
				MOVE_XorZ = -MOVE_XorZ;
			}
			bt_times = MOVE_XorZ > (temp / 1.2) ? 8 : MOVE_XorZ > (temp / 2) ? 16 : MOVE_XorZ > (temp / 4) ? 32 : MOVE_XorZ > (temp / 8) ? 64 : 100;
			if (uart4_recv_data[0] == 0x3B)
			{
				pdu[BatteryStatus] = 1;//< 电池读取成功
				pdu[BatteryVoltage] = uart4_recv_data[6] | (uart4_recv_data[7] << 8);//< 电池电压
				pdu[BatteryCurrent] = uart4_recv_data[8] | (uart4_recv_data[9] << 8);//< 电池电流
				pdu[BatteryQuantity] = (uart4_recv_data[10] | (uart4_recv_data[11] << 8)) * 2;//< 电池电量
				pdu[BatteryHealth] = uart4_recv_data[12] | (uart4_recv_data[13] << 8);//< 电池健康度
				pdu[BatteryTemperature] = (uart4_recv_data[24] | (uart4_recv_data[25] << 8)) - 2731;//< 电池温度
				pdu[BatteryProtectStatus] = uart4_recv_data[36] | (uart4_recv_data[37] << 8);//< 电池保护状态
			}
			break;
		default:
			bt_times = 100;//< 1000ms刷新1次
			if (uart4_recv_data[0] == 0x7e)
			{
				uint32_t tmp = 0;
				//uint16_t r_crc = (uint16_t)uart4_recv_data[uart4_recv_len - 2] | (((uint16_t)uart4_recv_data[uart4_recv_len - 1])<<8);
				//uint16_t c_crc = usMBCRC16(uart4_recv_data, uart4_recv_len - 2);			
				//if (r_crc == c_crc)
				// {
					int i = 0;
					pdu[BatteryStatus] = 1;//< 电池读取成功
					pdu[BatteryQuantity] = (uint16_t)uart4_recv_data[107] | (((uint16_t)uart4_recv_data[108]) << 8);//< 电池电量
					pdu[BatteryQuantity] *= 10;
					i = 97;
					tmp = (uint32_t)uart4_recv_data[i++];
					tmp |= (uint32_t)uart4_recv_data[i++] << 8;
					tmp |= (uint32_t)uart4_recv_data[i++] << 16;
					tmp |= (uint32_t)uart4_recv_data[i++] << 24;
					pdu[BatteryVoltage] = tmp*0.1;//< 电池电压
					tmp = (uint32_t)uart4_recv_data[i++];
					tmp |= (uint32_t)uart4_recv_data[i++] << 8;
					tmp |= (uint32_t)uart4_recv_data[i++] << 16;
					tmp |= (uint32_t)uart4_recv_data[i++] << 24;
					pdu[BatteryCurrent] = (int16_t)((int)tmp*0.1);//< 电池电流
					pdu[BatteryTemperature] = (uint16_t)uart4_recv_data[77] | (((uint16_t)uart4_recv_data[78]) << 8);//< 电池温度
				// }
			}
			break;
		}
	}
	bt++;
	if (bt >= bt_times)
	{
		bt = 0;
		GPIO_SetBits(USARTb_485en_GPIO, USARTb_485enPin);
		DMA_EnableChannel(USARTb_Tx_DMA_Channel, DISABLE);    // 关闭 DMA2 通道5, UART4_TX
		DMA_SetCurrDataCounter(USARTb_Tx_DMA_Channel, battery_send_frame_num);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
		DMA_EnableChannel(USARTb_Tx_DMA_Channel, ENABLE);    // 开启 DMA2 通道5, UART4_TX	
	}
}
//< 超声波检测
void UltrasonicProcess(void)
{
	if (ultrasonic_t1tig >= 1)
	{
		ultrasonic_t1tig++;
		if (ultrasonic_t1tig > 10)
		{
			int i = 0;
			ultrasonic_t1tig = 0;
			ultrasonic_t1tig_heartbeat = 0;
			pdu[Ultrasonic1_H] = (uint16_t)(ultrasonic_t1tig_time >> 16);
			pdu[Ultrasonic1_L] = (uint16_t)ultrasonic_t1tig_time;
			GPIO_SetBits(CS1_Ttig_PORT, CS1_Ttig_PIN);
			for (i = 0;i < 100;i++);
			GPIO_ResetBits(CS1_Ttig_PORT, CS1_Ttig_PIN);
			UltrasonicSetEnable(1, 1);
		}
	}
	else
	{
		ultrasonic_t1tig_heartbeat++;
		if (ultrasonic_t1tig_heartbeat > 50)
		{
			ultrasonic_t1tig = 1;
			ultrasonic_t1tig_heartbeat = 0;
		}
	}
	if (ultrasonic_t2tig >= 1)
	{
		ultrasonic_t2tig++;
		if (ultrasonic_t2tig >= 10)
		{
			int i = 0;
			ultrasonic_t2tig = 0;
			ultrasonic_t2tig_heartbeat = 0;
			pdu[Ultrasonic2_H] = (uint16_t)(ultrasonic_t2tig_time >> 16);
			pdu[Ultrasonic2_L] = (uint16_t)ultrasonic_t2tig_time;
			GPIO_SetBits(CS2_Ttig_PORT, CS2_Ttig_PIN);
			for (i = 0;i < 100;i++);
			GPIO_ResetBits(CS2_Ttig_PORT, CS2_Ttig_PIN);
			UltrasonicSetEnable(2, 1);
		}
	}
	else
	{
		ultrasonic_t2tig_heartbeat++;
		if (ultrasonic_t2tig_heartbeat > 50)
		{
			ultrasonic_t2tig = 1;
			ultrasonic_t2tig_heartbeat = 0;
		}
	}
}
//SPI读写函数
void  SPI1_ReadWriteByte(void)
{
	if (SPI_ReadWriteCycle == 0)
	{
		uint16_t retry = 0;
		GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_SET);
		pdu[exio_input_status] = (uint16_t)SPI_Master_Rx_Buffer;
		exio_input.input = pdu[exio_input_status];
		for (retry = 0;retry < 1000;)
		{
			retry++;
		}
		retry = 0;
		while ((SPI_MASTER->STS & 1 << 1) == 0) //等待发送区空
		{
			retry++;
			if (retry > 2000)
			{
				error_code = 11;//< SPI错误
				return;
			}
		}

		if (robot_control.bit.light_ctrl_en == 0)
		{//< 默认灯控制权
			pdu[light_control] = exio_output.output;
		}
		SPI_MASTER->DAT = pdu[light_control];
		SPI_ReadWriteCycle = 1;
		SPI_heartbeat = 0;
		pdu[car_light_messages] = pdu[light_control];
	}
	if (SPI_ReadWriteCycle == 1)
	{
		SPI_heartbeat++;
		if (SPI_heartbeat > 20)
		{
			SPI_heartbeat = 0;
			SPI_ReadWriteCycle = 0;
		}
	}
}

void Ultrasonic1_task(void* pvParameters)
{
	uint32_t tig_time = 0;
	int i = 0;
	uint32_t max_time;
	uint32_t min_time;
	pdu = (uint16_t*)pvParameters;
	while (1)
	{
		rt_thread_delay(1000);   //< 100ms

		GPIO_SetBits(CS1_Ttig_PORT, CS1_Ttig_PIN);
		for (i = 0;i < 100;i++);
		GPIO_ResetBits(CS1_Ttig_PORT, CS1_Ttig_PIN);
		tig_time = 0;
		i = 0;
		while (GPIO_ReadInputDataBit(CS1_Econ_PORT, CS1_Econ_PIN)== Bit_RESET)
		{
			tig_time++;
			if (tig_time >= 200000)
			{
				i = 1;
				break;
			}
		}
		if (i == 0)
		{
			max_time = 0;
			min_time = 100000000;
			tig_time = 0;
			while (GPIO_ReadInputDataBit(CS1_Econ_PORT, CS1_Econ_PIN) == Bit_SET)
			{
				tig_time++;
				if (tig_time >= 200000)
				{
					break;
				}
			}
		}
		else tig_time = 200000;
		ultrasonic1_filtering[f1_cnt] = tig_time;
		f1_cnt++;
		if (f1_cnt >= 4)f1_cnt = 0;
		ultrasonic_t1tig_time = 0;
		for (i = 0;i < 4;i++)
		{
			tig_time = ultrasonic1_filtering[i];
			ultrasonic_t1tig_time += tig_time;
			if (tig_time > max_time)max_time = tig_time;
			if (tig_time < min_time)min_time = tig_time;
		}
		ultrasonic_t1tig_time -= max_time;
		ultrasonic_t1tig_time -= min_time;
		ultrasonic_t1tig_time <<= 1;
		if (ultrasonic_t1tig_time > 400000)ultrasonic_t1tig_time = 400000;
		pdu[Ultrasonic1_H] = (uint16_t)(ultrasonic_t1tig_time >> 16);
		pdu[Ultrasonic1_L] = (uint16_t)ultrasonic_t1tig_time;


	}

}
void Ultrasonic2_task(void* pvParameters)
{
	uint32_t tig_time = 0;
	int i = 0;
	uint32_t max_time;
	uint32_t min_time;
	pdu = (uint16_t*)pvParameters;
	while (1)
	{
		rt_thread_delay(1000);   //< 100ms

		GPIO_SetBits(CS2_Ttig_PORT, CS2_Ttig_PIN);
		for (i = 0;i < 100;i++);
		GPIO_ResetBits(CS2_Ttig_PORT, CS2_Ttig_PIN);
		tig_time = 0;
		i = 0;
		while (GPIO_ReadInputDataBit(CS2_Econ_PORT, CS2_Econ_PIN) == Bit_RESET)
		{
			tig_time++;
			if (tig_time >= 200000)
			{
				i = 1;
				break;
			}
		}
		if (i == 0)
		{
			max_time = 0;
			min_time = 100000000;
			tig_time = 0;
			while (GPIO_ReadInputDataBit(CS2_Econ_PORT, CS2_Econ_PIN) == Bit_SET)
			{
				tig_time++;
				if (tig_time >= 200000)
				{
					break;
				}
			}
		}
		else tig_time = 200000;
		ultrasonic2_filtering[f2_cnt] = tig_time;
		f2_cnt++;
		if (f2_cnt >= 4)f2_cnt = 0;
		ultrasonic_t2tig_time = 0;
		for (i = 0;i < 4;i++)
		{
			tig_time = ultrasonic2_filtering[i];
			ultrasonic_t2tig_time += tig_time;
			if (tig_time > max_time)max_time = tig_time;
			if (tig_time < min_time)min_time = tig_time;
		}
		ultrasonic_t2tig_time -= max_time;
		ultrasonic_t2tig_time -= min_time;
		ultrasonic_t2tig_time <<= 1;
		if (ultrasonic_t2tig_time > 400000)ultrasonic_t2tig_time = 400000;
		pdu[Ultrasonic2_H] = (uint16_t)(ultrasonic_t2tig_time >> 16);
		pdu[Ultrasonic2_L] = (uint16_t) ultrasonic_t2tig_time;


	}

}

/****************************************************************
* 函数功能：车灯数据解析函数，使用VRA来控制，获取通道10来获取数据
* 返回值：
****************************************************************/

void Car_Light_Control(float Vx, float Vz)
{
	unsigned short usTemp = 0;
	usTemp = tagSBUS_CH.CH10;

	// 逻辑判断模块
	if (Abs_int(usTemp - rc_ptr->light_base) > 100)
	{// 此时代表没有触发开启灯光的标志		
		if ((usTemp - rc_ptr->light_base) > 0)
		{// 开启灯光			
			g_ucLightOnFlag = 1;
		}
		else
		{// 关闭灯光			
			g_ucLightOnFlag = 0;
		}
	}
	


	usTemp = tagSBUS_CH.CH9;//< 模拟软急停
	if (Abs_int(usTemp - 1023) > 100)
	{
		if ((usTemp - 1023) > 0)
		{// 开启软急停			
			emergency_stop.estop_soft = 1;			
		}
		else
		{// 关闭软急停	
			emergency_stop.estop_soft = 0;			
		}
		if (emergency_stop.estop_soft_old != emergency_stop.estop_soft)
		{
			emergency_stop.estop_soft_old = emergency_stop.estop_soft;
			if (emergency_stop.estop_soft)
			{
				emergency_stop.Descending = 1;
				GPIO_ResetBits(RJ_JT_GPIO, RJ_JT_Pin);//< 开启软急停	
			}
			else GPIO_SetBits(RJ_JT_GPIO, RJ_JT_Pin);//< 关闭软急停	
		}
	}

	if (g_eControl_Mode == CONTROL_MODE_UNKNOW)
	{//< 等待连接状态
		if (light_time.t_cnt_Light_Q++ >= 75)
		{
			if (exio_output.bit.Light_Q == 0)exio_output.output |= 0xf0;
			else exio_output.output &= 0x0f;
			light_time.t_cnt_Light_Q = 0;
		}
	}
	else
	{

		if (g_ucLightOnFlag == 1)exio_output.output |= 0xf0;
		else exio_output.output &= 0x0f;
		if(Vx>0)
		{//< 倒车
			exio_output.output |= 0xa0;
		}
		if (Vz >0)
		{//< 左转
			light_time.t_cnt_Light_Y = 0;
			light_time.t_cnt_Light_Q++;
			if (light_time.t_cnt_Light_Q <= 50)
			{
				exio_output.output |= 0x30;
			}
			else if (light_time.t_cnt_Light_Q <= 100)
			{
				exio_output.output &= ~0x30;
				if (light_time.t_cnt_Light_Q == 100)light_time.t_cnt_Light_Q = 0;
			}
			else light_time.t_cnt_Light_Q = 0;
			light_time.c_Light_Q = 1;
		}
		else if (Vz < 0)
		{//< 右转
			light_time.t_cnt_Light_Q = 0;
			light_time.c_Light_Q = 1;
			light_time.t_cnt_Light_Y++;
			if (light_time.t_cnt_Light_Y <= 50)
			{
				exio_output.output |= 0xc0;
			}
			else if (light_time.t_cnt_Light_Y <= 100)
			{
				exio_output.output &= ~0xc0;
				if (light_time.t_cnt_Light_Y == 100)light_time.t_cnt_Light_Y = 0;
			}
			else light_time.t_cnt_Light_Y = 0;
		}
		else
		{
			if (light_time.c_Light_Q)
			{
				light_time.c_Light_Q = 0;
				light_time.t_cnt_Light_Q = light_time.t_cnt_Light_Y = 0;
				if (g_ucLightOnFlag)exio_output.output |= 0xf0;
				else exio_output.output &= 0x0f;
			}
		}
	}
	if (emergency_stop.SW != exio_input.bit.X0)
	{
		if (exio_input.bit.X0)emergency_stop.Rising = 1;
		else emergency_stop.Descending = 1;
		emergency_stop.SW = exio_input.bit.X0;
	}
	if (exio_input.bit.X0|| emergency_stop.estop_soft)
	{
		if (light_time.t_cnt_RGB_G++ >= 15)
		{
			exio_output.bit.RGB_G = (exio_output.bit.RGB_G == 1) ? 0 : 1;
			exio_output.bit.RGB_R = exio_output.bit.RGB_G;
			light_time.t_cnt_RGB_G = 0;
			light_time.t_cnt_RGB_B++;
			if (light_time.t_cnt_RGB_B <= 6)
			{
				exio_output.bit.RGB_B = exio_output.bit.RGB_G;
			}
			else exio_output.bit.RGB_B = 0;
		}
		exio_output.bit.Light_Q = exio_output.bit.RGB_G;
		exio_output.bit.Light_Y = exio_output.bit.RGB_G;
	}
	else
	{
		if (emergency_stop.Descending)
		{
			emergency_stop.Descending = 0;
			exio_output .output &= ~0x7;
			light_time.t_cnt_RGB_G = 0;
			light_time.t_cnt_RGB_B = 0;
			light_time.t_cnt_RGB_G = 0;
		}
		if(g_eControl_Mode != CONTROL_MODE_UNKNOW)exio_output.bit.RGB_R = ((Vx == 0) && (Vz == 0));
	}
}

void BatteryInfoInit(void)
{
	int n = 0;
	uint8_t verifyADD8 = 0;
	battery_send_frame_num = 0;
	switch (pdu[battery_manufacturer])
	{
	case 1://< 深圳市锂神科技有限公司
		uart4_send_data[battery_send_frame_num++] = 0x3A;//< 帧头
		uart4_send_data[battery_send_frame_num++] = 0x7E;//< 通用：0x7E
		uart4_send_data[battery_send_frame_num++] = 0x01;//< 协议版本号
		uart4_send_data[battery_send_frame_num++] = 0x01;//< 0：写 1：读
		uart4_send_data[battery_send_frame_num++] = 0x1E;//< 功能码 (0x1E)
		uart4_send_data[battery_send_frame_num++] = 0x00;//< 长度
		uart4_send_data[battery_send_frame_num++] = 0xD8;//< 校验和
		battery_send_frame_num = 9;
		break;
	default://7E 0A 01 00 00 30 00 AC 00 00 2C 90 //< 电池信息初始化读取
		uart4_send_data[battery_send_frame_num++] = 0x7E;
		uart4_send_data[battery_send_frame_num++] = 0x0A;
		uart4_send_data[battery_send_frame_num++] = 0x01;
		uart4_send_data[battery_send_frame_num++] = 0x00;
		uart4_send_data[battery_send_frame_num++] = 0x00;
		uart4_send_data[battery_send_frame_num++] = 0x30;
		uart4_send_data[battery_send_frame_num++] = 0x00;
		uart4_send_data[battery_send_frame_num++] = 0xAC;
		uart4_send_data[battery_send_frame_num++] = 0x00;
		uart4_send_data[battery_send_frame_num++] = 0x00;
		uart4_send_data[battery_send_frame_num++] = 0x2C;
		uart4_send_data[battery_send_frame_num++] = 0x90;
		battery_send_frame_num = 14;
		break;
	}	
	uart4_send_flag = 2;//< 
}
/**************************************************************************
函数功能：MCU_INF_RX接收到来自小车的红外信号之后，开启充电桩的红外发送功能
入口参数：无
引脚信息：未接收到信号时，RX信号端低，ADCJT(PA0)高；
**************************************************************************/
void Sensor_TX_Control()
{
	//接收到红外信号
	if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO, MCU_INF_RX_PIN) == RESET)
	{
		//1启动红外传感器发送端
		MCU_INF_TX = 1;
	}
	else
	{
		MCU_INF_TX = 0;
	}
}
/*******************  *******************************************************
函数功能：红外通讯发送数据部分
入口参数：无/要发送的数据
引脚信息：发送信号端低电平时，TX信号端高，RGBG低；
**************************************************************************/

void IrDA_Guide(void)
{
	SendCout++;
	//高电平100ms
	if (SendCout <= 10)
	{
		MCU_INF_TX = 1;
		SendGuide_Flag = 0;
		BitFree = 0;
	}
	else if (SendCout <= 14)
	{
		MCU_INF_TX = 0;
	}
	else
	{
		SendCout = 0;
		SendGuide_Flag = 1;
		BitFree = 1;
	}

}
void IrDA_Send0(void)
{
	SendCout++;
	//高电平30ms
	if (SendCout <= 3)
	{
		MCU_INF_TX = 1;
		BitFree = 0;
	}
	else if (SendCout <= 10)
	{
		MCU_INF_TX = 0;
		BitFree = 0;
	}
	else
	{
		MCU_INF_TX = 0;
		SendCout = 0;
		BitFree = 1;
	}
}
void IrDA_Send1(void)
{
	SendCout++;
	//高电平70ms
	if (SendCout <= 7)
	{
		MCU_INF_TX = 1;
		BitFree = 0;
	}
	else if (SendCout <= 10)
	{
		MCU_INF_TX = 0;
		BitFree = 0;
	}
	else
	{
		MCU_INF_TX = 0;
		SendCout = 0;
		BitFree = 1;
	}
}
void IrDA_SendData(uint8_t SendData)
{
	if (SendGuide_Flag == 0)//0:未发送引导位；1：已发送引导位；
	{
		IrDA_Guide();
	}
	else
	{
		uint8_t bit = (SendData >> Send_i) & 0x01;
		if (bit)
		{
			IrDA_Send1();
			if (BitFree)//等待发送完成
			{
				Send_i--;//指针指向低位				
			}
		}
		else
		{
			IrDA_Send0();
			if (BitFree)//等待发送完成
			{
				Send_i--;//指针指向低位				
			}
		}
		if (Send_i == -1)
		{
			//数据发送完成
			Send_i = 3;//指针重置到数据高位
			SendGuide_Flag = 0;//置零引导位
		}
	}
}

void Relay_Switch(void)
{
	if (GPIO_ReadInputDataBit(MCU_CH_DET_GPIO, MCU_CH_DET_PIN) == RESET)
	{
		//开启继电器
		GPIO_SetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
		MCU_RELAY2 = 1;
	}
	else
	{
		GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
		MCU_RELAY2 = 0;
	}
}
/**************************************************************************
函数功能：核心控制相关
入口参数：
返回  值：
**************************************************************************/

void Balance_task(void* pvParameters)
{
	uint8_t tmp1 = 0;
	pdu = (uint16_t*)pvParameters;
	pdu[motor1_radius] = FourWheer_Radiaus * 10000;
	pdu[motor1_reduction_ratio] = REDUCTION_RATE * 100;
	float Radiaus = pdu[motor1_radius] * 0.0001;
	float rate = pdu[motor1_reduction_ratio] * 0.01;
	FourWheer_Perimeter = 2 * PI * Radiaus;//< 车轮周长
	FourWheer_Conversion = FourWheer_Perimeter / 60 / rate;	
	VelocityToRpmConversion = (60 * rate)/ FourWheer_Perimeter;
	AngularVelocityConversion = DIRECTOR_BASE* (PI / 4);
	pdu[car_default_mode] = g_eControl_Mode;
	pdu[wheel_distance] = (uint16_t)(Wheel_spacing * 10000);
	pdu[axles_distance] = (uint16_t)(Axle_spacing * 10000);
	pdu[motor_num] = Motor_Number;
	pdu[car_mode] = CONTROL_MODE_REMOTE;
	BatteryInfoInit();
	if (pdu[robot_acceleration] < 5000)pdu[robot_acceleration] = 5000;
	if (pdu[car_model] == Charger)
	{
		/*红外发送端初始化关闭*/
		IrDA_TX = 0;
		/*风扇初始化开启*/
		FAN1 = 1;
		FAN2 = 1;
		LimitSwitch_Init();
		Relay_Init();
		Key_Init();
		RGB_Init();
		LimitSwitch_Init();
		ChargeDetection_Init();
	}


	while (1)
	{
		rt_thread_delay(100);   //< 10ms
		if (tmp1 == 50)
		{
			if (g_emCarMode == Charger)
			LedBlink(LED2_PORT, RUN2);
			tmp1 = 0;
		}
		tmp1++;


		Get_Motor_Velocity();                  //获取驱动器反馈的速度，存放在结构体中
		SetReal_Velocity(pdu);                  //解析航模数据
		//使用上位机控制的时候，会在usart中断中关闭Remote_ON_Flag标志位
		if (g_eControl_Mode == CONTROL_MODE_UNKNOW)
		{
			Move_X = Move_Y = Move_Z = 0;
		}
		else
		{
			if (g_eControl_Mode == CONTROL_MODE_REMOTE||
				g_eControl_Mode == CONTROL_MODE_UART)
			{
				Remote_Control();                    //航模获取到设定的速度
			}
			else if (g_eControl_Mode == CONTROL_MODE_ROS)
			{
				// 上位机控制的时候，此时Z轴的速度要进行一个反向，不然左右转弯的时候有问题
				Ros_Control();
			}
			//小车线速度和角速度限幅
			float temp = ((int16_t)pdu[car_max_lin_speed]) * 0.001f;
			Move_X = Move_X > temp ? temp : Move_X;
			temp = ((int16_t)pdu[car_min_lin_speed]) * 0.001f;
			Move_X = Move_X < temp ? temp : Move_X;
			temp = ((int16_t)pdu[car_max_ang_speed]) * 0.001f;
			Move_Z = Move_Z > temp ? temp : Move_Z;
			temp = ((int16_t)pdu[car_min_ang_speed]) * 0.001f;
			Move_Z = Move_Z < temp ? temp : Move_Z;
		}
		pdu[car_current_ctrl_mode] = g_eControl_Mode;//
		switch (g_emCarMode)
		{
		case Mec_Car:        
			break; //麦克纳姆轮小车
		case Omni_Car:       
			break; //全向轮小车
		case Akm_Car:        
			break; //阿克曼小车
		case Diff_Car:       
			break; //两轮差速小车
		case FourWheel_Car://四驱车  室外差速
		{
			if (exio_input.bit.X0 || emergency_stop.estop_soft)
			{//< 急停
				Move_X = Move_Y = Move_Z = 0;
			}
			if (pdu[robot_forward_direction] == 1)Move_X = -Move_X;
			if (pdu[robot_turning_direction] == 1)Move_Z = -Move_Z;
			Drive_Motor(Move_X, Move_Y, Move_Z);   //小车运动模型解析出每个电机的实际速度
			Car_Light_Control(Move_X, Move_Z);
			Set_MotorVelocity(-MOTOR_A.nTarget_Velocity, -MOTOR_B.nTarget_Velocity,
				MOTOR_C.nTarget_Velocity, MOTOR_D.nTarget_Velocity);
		}
			break; 
		case Tank_Car:
			break; //履带车
		case RC_Car:
		{
			float m = (100+pdu[rc_magnification])*10;
			int p = (int)pdu[rc_speed_shifting];
			uint16_t line_ = Move_X  *m*0.5 + 3000 + p;
			p = (int)pdu[rc_angle_shifting];
			uint16_t angle_ = Move_Z *m + 3000+p;
			RCCAR_Process(line_, angle_);
		}
			break;
		case Charger:
		{
			/*红外发送端功能切换*/
			switch (IrDA_SendState)
			{//0：关闭发送端；1：红外对接；2：红外通讯；
			case 0:
				break;
			case 1:
				Sensor_TX_Control();
				//对接成功，转通讯	
				if (MCU_INF_TX == 1)
				{
					IrDA_SendState = 2;
				}
				break;
			case 2:
				IrDA_SendData(SendData);

				break;
			}
			
			//ReceiveData = IrDA_ReceiveData(pdu);
			ReceiveData = 1;
			/*红外接收端解码*/
			switch (ReceiveData)
			{
			case 0:
				break;
			case 0x01://红外对接正常
				SendData = 1;
				IrDA_SendData(SendData);
				//等待限位开关信号
				if (GPIO_ReadInputDataBit(MCU_SW_DET_GPIO, MCU_SW_DET_PIN) == RESET)
				{
					LimitSwitch_OK = 1;
					GPIO_ResetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);
				}
				else
				{
					GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);
					LimitSwitch_OK = 0;
				}
				break;
			case 0x02://关闭充电
				GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
				MCU_RELAY2 = 0;
				SendData = 2;
				IrDA_SendData(SendData);
				break;
			case 0x03://充电异常
				GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
				MCU_RELAY2 = 0;
				SendData = 3;
				IrDA_SendData(SendData);
				break;
			default:
				break;
			}
			//按键切换RGB颜色
			Key_Change_RGB();
			Relay_Switch();
		}
		break;
		default: break;
		}
		
		pdu[car_mode] = (pdu[car_mode] & 0xFF00) | g_eControl_Mode;
		pdu[error_get_and_clear] = error_code;//< 故障获取
		pdu[car_error_messages] = error_code;
		if (error_code != ERROR_NONE)
		{
			pdu[car_system_state] = 2;
		}
		BatteryInformation();
		SPI1_ReadWriteByte();
		PowerControl();
		//UltrasonicProcess();
	}
}
