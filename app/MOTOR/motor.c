#include "motor_data.h"
#include "user_can.h"
#include "485_address.h"
#include "robot_select_init.h"
#include "system.h"
#include "motor.h"
#include "mb.h"
#include "usartx.h"
#include "balance.h"
#include "math.h"
#include "bsp.h"
#include "RJ_JT.h"

int Servo_pulse =0;	//位置伺服脉冲

uint32_t DiffposForOdom = 0;//里程计当次差距
int LastPos_Left = 0;//左轮上次位置
int LastPos_Right = 0;//右轮上次位置
int NowPos_Left = 0;//左轮当前位置
int NowPos_Right = 0;//右轮当前位置

uint32_t Odom_distance_mm = 0;//里程计(单位mm)
float TireCircumference = 0;//轮胎周长->轮胎半径,
float MotorSpeedToLineSpeed= 0;//电机转速->线速度,
float LineSpeedToMotorSpeed= 0;//线速度->电机转速
float MotorSpeedZLACToPulse= 0;//速度反馈单位转换
float MotorPulseWANZEToMotorSpeed = 0;//速度反馈单位转换

#define VELOCITY_LIMIT       0x0005

bool* TPDOMessage_FLAG[TPDO_GROUP_COUNT][MAX_MOTOR_NUMBER] = {{&M1T0Message_FLAG, &M2T0Message_FLAG, &M3T0Message_FLAG, &M4T0Message_FLAG, &M5T0Message_FLAG,
											&M6T0Message_FLAG, &M7T0Message_FLAG, &M8T0Message_FLAG, &M9T0Message_FLAG, &M10T0Message_FLAG},
															  {&M1T1Message_FLAG, &M2T1Message_FLAG, &M3T1Message_FLAG, &M4T1Message_FLAG, &M5T1Message_FLAG,
									 		&M6T1Message_FLAG, &M7T1Message_FLAG, &M8T1Message_FLAG, &M9T1Message_FLAG, &M10T1Message_FLAG}, 
															  {&M1T2Message_FLAG, &M2T2Message_FLAG, &M3T2Message_FLAG, &M4T2Message_FLAG, &M5T2Message_FLAG,
									 		&M6T2Message_FLAG, &M7T2Message_FLAG, &M8T2Message_FLAG, &M9T2Message_FLAG, &M10T2Message_FLAG}};

CanRxMessage* TPDOMessage[TPDO_GROUP_COUNT][MAX_MOTOR_NUMBER] = {{&M1T0Message, &M2T0Message, &M3T0Message, &M4T0Message, &M5T0Message, 
													&M6T0Message, &M7T0Message, &M8T0Message, &M9T0Message, &M10T0Message},
															   	 {&M1T1Message, &M2T1Message, &M3T1Message, &M4T1Message, &M5T1Message, 
													&M6T1Message, &M7T1Message, &M8T1Message, &M9T1Message, &M10T1Message}, 
															   	 {&M1T2Message, &M2T2Message, &M3T2Message, &M4T2Message, &M5T2Message, 
													&M6T2Message, &M7T2Message, &M8T2Message, &M9T2Message, &M10T2Message}};

/**************************************************************************
函数功能：电机相关参数初始化
入口参数：机器人类型 
返回  值：无
**************************************************************************/
void InitMotorParameters(void)
{
	//轮胎半径->轮胎周长, 单位：m	公式：C=2 * PI * R 
	TireCircumference = 2 * PI * pdu[wheel_radius] * MAGNIFIC_10000x_DOWN;
	//电机转速->线速度, 单位：m/s	公式： C / REDUCTION_RATE / 60
	MotorSpeedToLineSpeed = TireCircumference / pdu[motor1_reduction_ratio] / MINTOSEC;
	//线速度->电机转速, 单位：r/min	公式： REDUCTION_RATE * 60 / C
	LineSpeedToMotorSpeed = 1 / MotorSpeedToLineSpeed;
	//速度反馈单位转换0.1r/min->p/s(ZLAC), 	公式： 0.1 / 60 * ENCODER_LINES
	MotorSpeedZLACToPulse = MAGNIFIC_10x_DOWN / MINTOSEC * ENCODER_LINES;
	//速度反馈单位转换p/s->0.1r/min(WANZE), 单位：r/min	公式： REDUCTION_RATE * 60 / C
	MotorPulseWANZEToMotorSpeed = 1 / MotorSpeedZLACToPulse;	
	//角度最大值->弧度最大值，单位：0.1 degree -> radian
	Z_Radian_Max = pdu[max_angle] * MAGNIFIC_10x_DOWN * ANGtoRAD;	
}	

/**************************************************************************
函数功能：电机刷新任务
入口参数：无 
返回  值：无
**************************************************************************/
void Motor_task(void* pvParameters)
{
	while (1){
		rt_thread_delay(50);//此任务以5ms的频率运行
		Detect_Motor_Status();
		Set_MotorVelocity();
	}
}

/**********************************************************
 * 函数功能： 接收TPDO、检查电机是否在线
 * 说    明： 将TPDO接收到的消息存放在结构体中。
 *            如果电机掉线，则需要重新初始化电机。
 **********************************************************/
void Detect_Motor_Status(void)
{
	int i, j, k;
	uint16_t type, sport_mode, state_word, temperature, voltage, current, error_code,
			position1_feedback, position2_feedback, rpm_feedback, torgue_feedback;//后面有解释
	uint16_t mpl1, mpl2, mpr1, mpr2, ms1, ms2;//后面要用到的临时变量
	int rpm_feedback_temp;//后面有解释
	for (i = 0; i < pdu[TpdoGroupCount]; i++){
		for (j = 0; j < pdu[motor_number]; j++){
			if(*TPDOMessage_FLAG[i][j]){
				*TPDOMessage_FLAG[i][j] = false;
				type = motor1_type + j * pdu[ro_motor_gap];
				sport_mode = motor1_sport_mode + j * pdu[ro_motor_gap];
				state_word = motor1_state_word + j * pdu[ro_motor_gap];
				temperature = motor1_temperature + j * pdu[ro_motor_gap];
				voltage = motor1_voltage + j * pdu[ro_motor_gap];
				current = motor1_current + j * pdu[ro_motor_gap];
				error_code = motor1_error_code + j * pdu[ro_motor_gap];
				position1_feedback = motor1_position1_feedback + j * pdu[ro_motor_gap];
				position2_feedback = motor1_position2_feedback + j * pdu[ro_motor_gap];
				rpm_feedback = motor1_rpm_feedback + j * pdu[ro_motor_gap];
				torgue_feedback = motor1_torgue_feedback + j * pdu[ro_motor_gap];
				k = 0;
				mtd[j].d.none_count = 0;
				mtd[j].d.online = true;
				switch (i){
					case TPDO0:
						switch (pdu[type]){
							case servo_zlac:
							case servo_wanze:
								pdu[state_word]  = TPDOMessage[i][j]->Data[k++];					//状态字
								pdu[state_word] |= TPDOMessage[i][j]->Data[k++] << 8;	
								pdu[position1_feedback]  = TPDOMessage[i][j]->Data[k++];			//当前位置（低字节）
								pdu[position1_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[position2_feedback]  = TPDOMessage[i][j]->Data[i++];			//当前位置（高字节）
								pdu[position2_feedback] |= TPDOMessage[i][j]->Data[i++] << 8;	
								pdu[temperature]  = TPDOMessage[i][j]->Data[k++];				//电机PCB温度
								pdu[temperature] |= TPDOMessage[i][j]->Data[k++] << 8;
								break;
							case servo_zlacd:
								pdu[state_word]  = TPDOMessage[i][j]->Data[k++];					//状态字
								pdu[state_word] |= TPDOMessage[i][j]->Data[k++] << 8;	
								pdu[motor1_rpm_feedback]  = TPDOMessage[i][j]->Data[k++];			//左电机
								pdu[motor1_rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[motor2_rpm_feedback]  = TPDOMessage[i][j]->Data[k++];			//右电机
								pdu[motor2_rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								break;				
							case servo_plan:
								/* code */
								break;	
						}
					case TPDO1:
						switch (pdu[type]){
							case servo_zlac:
								pdu[rpm_feedback]  = TPDOMessage[i][j]->Data[k++];				//当前速度
								pdu[rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								k += 2;//舍去k = 2, k = 3数据
								switch (pdu[sport_mode]){
									case torque_mode:
										pdu[torgue_feedback]  = TPDOMessage[i][j]->Data[k++];		//当前转矩
										pdu[torgue_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;									
										break;
									default:
										pdu[voltage]  = TPDOMessage[i][j]->Data[k++];				//母线电压
										pdu[voltage] |= TPDOMessage[i][j]->Data[k++] << 8;	
										break;
								}
								pdu[error_code]  = TPDOMessage[i][j]->Data[k++];					//错误代码
								pdu[error_code] |= TPDOMessage[i][j]->Data[k++] << 8;
							case servo_wanze:
								rpm_feedback_temp  = TPDOMessage[i][j]->Data[k++];					//当前速度（脉冲）
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 8;
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 16;			
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 24;
								rpm_feedback_temp = (int)(rpm_feedback_temp * MotorPulseWANZEToMotorSpeed);
								pdu[rpm_feedback]  = rpm_feedback_temp & 0xFFFF;					//当前速度（0.1rpm）		
								pdu[voltage]  = TPDOMessage[i][j]->Data[k++];						//母线电压
								pdu[voltage] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[current]  = TPDOMessage[i][j]->Data[k++];						//母线电流
								pdu[current] |= TPDOMessage[i][j]->Data[k++] << 8;
								break;	
							case servo_zlacd:
								/* NULL */
								break;			
							case servo_plan:
								/* code */
								break;				
						}
					case TPDO2:
						/* code */
						break;			
				}
			}
		}
	}
	for(int count = 0; count < pdu[motor_number]; count++){								//更新none_count		
		if(mtd[count].d.none_count < 10000){
			mtd[count].d.none_count ++;
		}
		if(mtd[count].d.none_count > Offline_Time_1s || pdu[can_reinitialize] == 5){	//1s内无PDO回复或者重置电机CAN通信
			mtd[count].d.online = false;												//电机在线状态为掉线
			Motor_Init(count);	
		}
	}
	pdu[can_reinitialize] = 0;

	switch (pdu[car_type]){
		case Akm_Car:
		case FourWheel_Car:
	 		mpl1 = pdu[motor2_position1_feedback];
			mpl2 = pdu[motor2_position2_feedback];
			mpr1 = pdu[motor3_position1_feedback];
			mpr2 = pdu[motor3_position2_feedback];
			ms1 = pdu[motor2_rpm_feedback];
			ms2 = pdu[motor3_rpm_feedback];
			break;
		case Diff_Car:
		case TwoWheel_Car:
		case Tank_Car:
	 		mpl1 = pdu[motor1_position1_feedback];
			mpl2 = pdu[motor1_position2_feedback];
			mpr1 = pdu[motor2_position1_feedback];
			mpr2 = pdu[motor2_position2_feedback];
			ms1 = pdu[motor1_rpm_feedback];
			ms2 = pdu[motor2_rpm_feedback];
			break;
	}


	//计算位移差，公式：脉冲数/一圈脉冲数/减速比*轮胎周长(m)*1000 = mm 	
	if(pdu[control_mode] == control_mode_ros){//控制模式是ROS
		NowPos_Left 	= (int)((uint32_t)mpl2 << 16 | mpl1);
		NowPos_Right 	= (int)((uint32_t)mpr2 << 16 | mpr1);		
		DiffposForOdom 	= (uint32_t)(round((float)((myabs(NowPos_Left - LastPos_Left) + myabs(NowPos_Right - LastPos_Right)) / 2) 
									/ pdu[motor2_reduction_ratio] / ENCODER_LINES * TireCircumference * MAGNIFIC_1000x_UP));//最后单位:mm
		Odom_distance_mm += DiffposForOdom;
		pdu[Odom1ForRobot] = Odom_distance_mm & 0xFFFF;			//里程计记录
		pdu[Odom2ForRobot] = (Odom_distance_mm >> 16) & 0xFFFF;	//里程计记录
		LastPos_Left = NowPos_Left;
		LastPos_Right = NowPos_Right;
	}
	if((short)pdu[ms1] > 0){//根据电机的速度方向来判断运行方向
		pdu[linear_speed_feedback]	= (short)(((short)ms1 + ((short)(0xFFFF - ms2 + 1) & 0xFFFF)) / 2 * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP);//线速度反馈(0.1r/min -> mm/s)
	}else{
		pdu[linear_speed_feedback]	= -(short)(((short)ms2 + ((short)(0xFFFF - ms1 + 1) & 0xFFFF)) / 2 * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP);//线速度反馈(0.1r/min -> m/s)
	}	
	pdu[yaw_speed_feedback] = -(short)(((float)(ms1 - ((0xFFFF - ms2 + 1) & 0xFFFF)) * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP) / ((float)pdu[car_tread] * MAGNIFIC_10000x_DOWN));//角速度反馈
	// pdu[yaw_speed_feedback] = -(short)(((float)(pdu[motor2_rpm_feedback] - ((0xFFFF - pdu[motor3_rpm_feedback] + 1) & 0xFFFF)) / 10 * MotorSpeedToLineSpeed * 1000) / ((float)pdu[car_tread] / 10000));//角速度反馈
	//角速度公式错误，等待角度可求之后，再用正确公式求出角速度
}

/**************************************************************************
函数功能：设置电机速度
入口参数：参数分别代表的是对应的can从机id和电机速度
返 回 值：无
**************************************************************************/
void Set_MotorVelocity(void)
{
	uint16_t Car_Error_Label = 0; //电机错误标志
	static short last_nMotor[MAX_MOTOR_NUMBER] = {0};
	short diff_nMotor[MAX_MOTOR_NUMBER];
	for (int count = 0; count < pdu[motor_number]; count++) {	
		uint16_t x_sport_mode = motor1_sport_mode + count * pdu[ro_motor_gap];
		uint16_t x_enstate = motor1_enable_state + count * pdu[ro_motor_gap];
		uint16_t x_node_state = motor1_node_state + count * pdu[ro_motor_gap];
		uint16_t x_error_code = motor1_error_code + count * pdu[ro_motor_gap];
		uint16_t x_direction = motor1_direction + count * pdu[rw_motor_gap];
		uint16_t x_target_rpm = motor1_target_rpm + count * pdu[rw_motor_gap];
		uint16_t x_target_torque = motor1_target_torque + count * pdu[rw_motor_gap];
		if(mtd[count].d.online){
			switch (pdu[x_sport_mode]){
				case position_mode:
					/* code */
					mrd[count].d.target_pos_vel = Servo_pulse;
					break;
				case speed_mode:
					if(pdu[x_enstate] && pdu[x_target_rpm]){
						pdu[x_node_state] = node_running;		
					}else{	
						pdu[x_node_state] = node_standby;
					}
					if(car_type == Diff_Car){
						diff_nMotor[0] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[motor1_target_rpm] - last_nMotor[0]), VELOCITY_LIMIT));
						diff_nMotor[1] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[motor2_target_rpm] - last_nMotor[1]), VELOCITY_LIMIT));
						last_nMotor[0] = mrd[0].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[0];//方向调整	
						last_nMotor[1] = mrd[1].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[1];//方向调整					
					}else{
						diff_nMotor[count] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[x_target_rpm] - last_nMotor[count]), VELOCITY_LIMIT));
						last_nMotor[count] = mrd[count].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[count]; //方向调整
					}
					break;
				case torque_mode:
					if(pdu[x_enstate] && pdu[x_target_torque]){
						pdu[x_node_state] = node_running;		
					}else{	
						pdu[x_node_state] = node_standby;
					}
					if(car_type == Diff_Car){
						diff_nMotor[0] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[motor1_target_torque] - last_nMotor[0]), VELOCITY_LIMIT));
						diff_nMotor[1] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[motor2_target_torque] - last_nMotor[1]), VELOCITY_LIMIT));
						last_nMotor[0] = mrd[0].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[0];//方向调整	
						last_nMotor[1] = mrd[1].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[1];//方向调整					
					}else{
						diff_nMotor[count] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[x_target_torque] - last_nMotor[count]), VELOCITY_LIMIT));
						last_nMotor[count] = mrd[count].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[count]; //方向调整
					}
					break;
			}
		}else{
			Car_Error_Label++; //每当有一个电机不在线 就累加一次
			mrd[count].d.target_pos_vel = 0;
			if(pdu[x_error_code]){
				pdu[x_node_state] = node_error;	
			}else{
				pdu[x_node_state] = node_disconnect;
			}	
		}		
	}
	pdu[car_running_state] = Judge_CarState(Car_Error_Label, Soft_JT_Flag);
}

/**********************************************************
 * 函数功能： 根据节点状态和输入信号判断车辆状态
 * 参数：     三个电机状态、状态输入点、手柄急停标志位
 * 返回值：   返回 CarRunningState 值。
 **********************************************************/
enum enum_car_running_state Judge_CarState(uint16_t Car_Error_Label, bool Soft_JT_Flag) 
{
	switch(Car_Error_Label){
		case 0:		pdu[car_error_messages] = car_normal;				break;
		case 1:		pdu[car_error_messages] = single_motor_error;		break;
		default:	pdu[car_error_messages] = multiple_motor_errors;	break;			
	}

	if(Car_Error_Label)				return	car_error;
	else if(Soft_JT_Flag) 			return	car_emergency_stop;
	else if(pdu[target_linear_speed] || pdu[target_yaw_speed] || pdu[target_angle])	
									return	car_running;
	else 							return	car_standby;
}

/**********************************************************
 * 函数功能： 电机初始化。
 * 参数：     ID为从机地址。
 * 说明：     无。
 **********************************************************/
void Motor_Init(int count)
{		
	uint16_t i_start = (count == pdu[motor_number])? 0 : count;
	uint16_t i_end = (count == pdu[motor_number])? pdu[motor_number] : (count + 1);
	uint16_t mode, id, ah, al, dh, dl;
	for (int i = i_start; i < i_end; i++) {	//初始化MOTOR参数
		mode = pdu[motor1_sport_mode + i * pdu[ro_motor_gap]];
		id = pdu[motor1_CAN_id + i * pdu[rw_motor_gap]];
		ah = (pdu[motor1_acceleration_time + i * pdu[rw_motor_gap]] >> 8) & 0x00FF;
		al = pdu[motor1_acceleration_time + i * pdu[rw_motor_gap]] & 0x00FF;
		dh = (pdu[motor1_deceleration_time + i * pdu[rw_motor_gap]] >> 8) & 0x00FF;
		dl = pdu[motor1_deceleration_time + i * pdu[rw_motor_gap]] & 0x00FF;
		switch(mode){
			case servo_zlac:
				ZLAC8015_PDO_Config(id, mode, ah, al, dh, dl);//电机PDO配置			
				Driver_JT_Inv(id);			//反转电平	
				break;
			case servo_wanze:
				WANZER_PDO_Config(id, mode);
				break;	
			case servo_zlacd:
				ZLAC8015D_PDO_Config(id, mode, ah, al, dh, dl);
				break;			
			case servo_plan:
				PLAN_PDO_Config(id, mode, ah, al, dh, dl);
				break;
			default:
				break;
		}
		NMT_Control(Start_Command, id); 			//启动命令
	}			
}

/**********************************************************
 * 函数功能： 通用的限幅函数。
 * 参数：     无。
 * 说明：     无。
 **********************************************************/
short limit_value(short value, short max_value) 
{
    return fmax(-max_value, fmin(value, max_value));
}

/**********************************************************
 * 函数功能： can1中断服务函数
 * 参数：     无
 * 说明：     通过检测TPDO消息来判断从机是否在线
 **********************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN1);
}

/**********************************************************
 * 函数功能： can2中断服务函数
 * 参数：     无
 * 说明：     通过检测TPDO消息来判断从机是否在线
 **********************************************************/
void CAN2_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN2);
}

/**********************************************************
 * 函数功能： can中断服务函数
 * 参数：     无
 * 说明：     通过检测TPDO消息来判断从机是否在线
 **********************************************************/
bool CanIRQProcess(CAN_Module* CANx)
{
	CanRxMessage RxMessage;
	CAN_ReceiveMessage(CANx, 0, &RxMessage);
	for (int i = 0; i < pdu[TpdoGroupCount]; i++){
		for (int j = 0; j < pdu[motor_number]; j++){	
			uint16_t TPDOx_ID = TPDO0_ID + i * (TPDO1_ID - TPDO0_ID);
			uint16_t x_id = pdu[motor1_CAN_id + j * pdu[rw_motor_gap]];
			if(RxMessage.StdId == TPDOx_ID + x_id){
				rt_memcpy(TPDOMessage[i][j], &RxMessage, sizeof(RxMessage));	
				*TPDOMessage_FLAG[i][j] = true;
				CAN_ClearFlag(CANx, CAN_INT_FMP0);
				return true;
			}
		}
	}
	CAN_ClearFlag(CANx, CAN_INT_FMP0);
	return false;
}




