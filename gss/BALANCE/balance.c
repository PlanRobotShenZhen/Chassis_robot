#include "balance.h"
#include "user_can.h"
#include "motor_data.h"
#include "485_address.h"

struct Smooth_Control tagSmooth_control;
float Move_X = 0,Move_Y = 0,Move_Z = 0;
static uint16_t* pdu;
uint16_t error_code = ERROR_NONE;
float FourWheer_Perimeter;
float FourWheer_Conversion;
float VelocityToRpmConversion;//速度换算电机转速
float AngularVelocityConversion;//角速度换算



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
	int i = car_max_lin_speed; //起始索引
	//小车线速度和角速度限幅
	tmp.ud[1] = pdu[i++];
	tmp.ud[0] = pdu[i++];
	Move_X = Move_X > tmp.v ? tmp.v : Move_X;
	tmp.ud[1] = pdu[i++];
	tmp.ud[0] = pdu[i++];
	Move_X = Move_X < tmp.v ? tmp.v : Move_X;
	tmp.ud[1] = pdu[i++];
	tmp.ud[0] = pdu[i++];
	Move_Z = Move_Z > tmp.v ? tmp.v : Move_Z;
	tmp.ud[1] = pdu[i++];
	tmp.ud[0] = pdu[i++];
	Move_Z = Move_Z < tmp.v ? tmp.v : Move_Z;
	//更新小车的目标线速度和角速度
	tmp.v = Move_X;
	i = 158;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = Move_Z;
	pdu[i++] = tmp.ud[1];
	pdu[i] = tmp.ud[0];
	//四驱车 ---- 当前所用到的车
	if(g_emCarMode == FourWheel_Car) 
	{	
		//用的时候motora和motorb要设置的为负数
		float tmp_value = Vz * (Wheel_spacing +  Axle_spacing) / 2.0f;
		//MOTOR_A.fltTarget_velocity  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左后轮的目标速度
		//MOTOR_B.fltTarget_velocity = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左前轮的目标速度
		//MOTOR_C.fltTarget_velocity  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右前轮的目标速度
		//MOTOR_D.fltTarget_velocity  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右后轮的目标速度
		MOTOR_A.fltTarget_velocity = Vx + tmp_value;
		MOTOR_B.fltTarget_velocity = MOTOR_A.fltTarget_velocity; //计算出左前轮的目标速度
		MOTOR_C.fltTarget_velocity = Vx - tmp_value;
		MOTOR_D.fltTarget_velocity = MOTOR_C.fltTarget_velocity;
		//更新四个电机的目标速度
		i = motor1_tar_speed;
		int length = motor2_tar_speed-motor1_tar_speed;
		tmp.v = MOTOR_A.fltTarget_velocity;
		pdu[i] = tmp.ud[1];
		pdu[i+1] = tmp.ud[0];
		i = i + length;
		tmp.v = MOTOR_B.fltTarget_velocity;
		pdu[i] = tmp.ud[1];
		pdu[i+1] = tmp.ud[0];
		i = i + length;
		tmp.v = MOTOR_C.fltTarget_velocity;
		pdu[i] = tmp.ud[1];
		pdu[i+1] = tmp.ud[0];
		i = i + length;
		tmp.v = MOTOR_D.fltTarget_velocity;
		pdu[i] = tmp.ud[1];
		pdu[i+1] = tmp.ud[0];

		int tva = SpeedVelocityToRotate(MOTOR_A.fltTarget_velocity); //转换为r/min
		// int tvb = 500 * SpeedVelocityToRotate(MOTOR_B.fltTarget_velocity);
		int tvb = tva;
		int tvc = SpeedVelocityToRotate(MOTOR_C.fltTarget_velocity);
		//int tvd = 500 * SpeedVelocityToRotate(MOTOR_D.fltTarget_velocity);
		int tvd = tvc;

		int diff = tva - MOTOR_A.nTarget_Velocity;
		int acc = 2500;
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
	int nMotor_A = mtd[0].d.current_velocity;
	int nMotor_B = mtd[1].d.current_velocity;
	int nMotor_C = mtd[2].d.current_velocity;
	int nMotor_D = mtd[3].d.current_velocity;	

	MOTOR_A.nFeedback_Velocity = nMotor_A;
	MOTOR_B.nFeedback_Velocity = nMotor_B;
	MOTOR_C.nFeedback_Velocity = nMotor_C;
	MOTOR_D.nFeedback_Velocity = nMotor_D;
	
	MOTOR_A.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_A);
	MOTOR_B.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_B);
	MOTOR_C.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_A);
	MOTOR_D.fltFeedBack_Velocity = RotateToSpeedVelocity(nMotor_B);
	union {
		float v;
		int16_t ud[2];
	}tmp;
	int i = motor1_feedback_speed;
	int length = motor2_feedback_speed-motor1_feedback_speed;
	tmp.v = MOTOR_A.fltFeedBack_Velocity;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
	i=i+length;
	tmp.v = MOTOR_B.fltFeedBack_Velocity;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
	i=i+length;
	tmp.v = MOTOR_C.fltFeedBack_Velocity;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
	i=i+length;
	tmp.v = MOTOR_D.fltFeedBack_Velocity;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
	i = car_feedback_lin_speed;
	tmp.v = (MOTOR_A.fltFeedBack_Velocity+MOTOR_B.fltFeedBack_Velocity+MOTOR_C.fltFeedBack_Velocity+MOTOR_D.fltFeedBack_Velocity)/4;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
	tmp.v = (-MOTOR_B.fltFeedBack_Velocity - MOTOR_A.fltFeedBack_Velocity + MOTOR_C.fltFeedBack_Velocity + MOTOR_D.fltFeedBack_Velocity)/2/(pdu[wheel_distance]+pdu[axles_distance])/10000;
	pdu[i] = tmp.ud[1];
	pdu[i+1] = tmp.ud[0];
}


/**************************************************************************
函数功能：将电机的转速，r/min转换成轮子线速度 m/s，
					存放在速度结构体中。
					1 r/min = （2Π * R） / 60  m/s
入口参数：nRotateSpeed代表转速，r/min
返 回 值：轮子转速
**************************************************************************/
float RotateToSpeedVelocity(int nRotateSpeed)
{
	//float fltReturn = 0.0f;
	//v=n*2*pi*r/60
	//fltReturn = nRotateSpeed * 2 * PI * FourWheer_Radiaus/ 60;  //转换为轮胎速度
	// 2 * PI * FourWheer_Radiaus/ 60 = 0.0172787593
	//fltReturn = nRotateSpeed * 0.0172787593;  //转换为轮胎速度
	return nRotateSpeed * FourWheer_Perimeter/60;  //转换为轮胎速度;	
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
函数功能：按键即时更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Update_Gyroscope(void)
{	
	memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));//双击更新陀螺仪零点
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
}

/*---------------------------一些功能函数--------------------------------*/
/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
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



/**************************************************************************
函数功能：核心控制相关
入口参数：
返回  值：
**************************************************************************/
void Balance_task(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt(); //获取到系统上次执行时间
	pdu = getPDUData();
	// pdu[61] 电机1车轮半径 pdu[62] 电机1减速比
	//FourWheer_Perimeter = 2 * PI * FourWheer_Radiaus;
	union {
		float v;
		int16_t ud[2];
	}tmp;
	int i = car_max_lin_speed; //起始索引
	tmp.v = 0.802;//< 机器人最大线速度初始化
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.802;//< 机器人最小线速度
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = 0.8014;//< 机器人最大角速度
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.8014;//< 机器人最小角速度
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];


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
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); //此任务以100Hz的频率运行（10ms控制一次）

		g_eControl_Mode = pdu[car_mode]&0xff;
		Get_Motor_Velocity();                  //获取驱动器反馈的速度，存放在结构体中
		SetReal_Velocity(pdu);                  //解析航模数据
		//使用上位机控制的时候，会在usart中断中关闭Remote_ON_Flag标志位
		if (g_eControl_Mode == CONTROL_MODE_UNKNOW)
		{
			motorA_ptr->motor_state = 0;
			motorB_ptr->motor_state = 0;
			motorC_ptr->motor_state = 0;
			motorD_ptr->motor_state = 0;
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
			motorA_ptr->motor_state = 1;
			motorB_ptr->motor_state = 1;
			motorC_ptr->motor_state = 1;
			motorD_ptr->motor_state = 1;
			
		}
		pdu[CONTROL_MODE_ADDR] = g_eControl_Mode;//
		Drive_Motor(Move_X, Move_Y, Move_Z);   //小车运动模型解析出每个电机的实际速度

		switch (g_emCarMode)
		{
		case Mec_Car:        break; //麦克纳姆轮小车
		case Omni_Car:       break; //全向轮小车
		case Akm_Car:        break; //阿克曼小车
		case Diff_Car:       break; //两轮差速小车
		case FourWheel_Car:
			Set_MotorVelocity(-MOTOR_A.nTarget_Velocity, -MOTOR_B.nTarget_Velocity,
				MOTOR_C.nTarget_Velocity, MOTOR_D.nTarget_Velocity);

			break; //四驱车 
		case Tank_Car:
			break; //履带车

		default: break;
		}
		
		pdu[car_mode] = (pdu[car_mode] & 0xFF00) | g_eControl_Mode;
		pdu[error_get_and_clear] = error_code;//< 故障获取
		pdu[car_error_messages] = error_code;
		if (error_code != ERROR_NONE)
		{
			pdu[car_system_state] = 2;
		}
	}
}
