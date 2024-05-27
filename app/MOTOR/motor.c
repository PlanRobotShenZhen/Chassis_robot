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

int Servo_pulse =0;	//λ���ŷ�����

uint32_t DiffposForOdom = 0;//��̼Ƶ��β��
int LastPos_Left = 0;//�����ϴ�λ��
int LastPos_Right = 0;//�����ϴ�λ��
int NowPos_Left = 0;//���ֵ�ǰλ��
int NowPos_Right = 0;//���ֵ�ǰλ��

uint32_t Odom_distance_mm = 0;//��̼�(��λmm)
float TireCircumference = 0;//��̥�ܳ�->��̥�뾶,
float MotorSpeedToLineSpeed= 0;//���ת��->���ٶ�,
float LineSpeedToMotorSpeed= 0;//���ٶ�->���ת��
float MotorSpeedZLACToPulse= 0;//�ٶȷ�����λת��
float MotorPulseWANZEToMotorSpeed = 0;//�ٶȷ�����λת��

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
�������ܣ������ز�����ʼ��
��ڲ��������������� 
����  ֵ����
**************************************************************************/
void InitMotorParameters(void)
{
	//��̥�뾶->��̥�ܳ�, ��λ��m	��ʽ��C=2 * PI * R 
	TireCircumference = 2 * PI * pdu[wheel_radius] * MAGNIFIC_10000x_DOWN;
	//���ת��->���ٶ�, ��λ��m/s	��ʽ�� C / REDUCTION_RATE / 60
	MotorSpeedToLineSpeed = TireCircumference / pdu[motor1_reduction_ratio] / MINTOSEC;
	//���ٶ�->���ת��, ��λ��r/min	��ʽ�� REDUCTION_RATE * 60 / C
	LineSpeedToMotorSpeed = 1 / MotorSpeedToLineSpeed;
	//�ٶȷ�����λת��0.1r/min->p/s(ZLAC), 	��ʽ�� 0.1 / 60 * ENCODER_LINES
	MotorSpeedZLACToPulse = MAGNIFIC_10x_DOWN / MINTOSEC * ENCODER_LINES;
	//�ٶȷ�����λת��p/s->0.1r/min(WANZE), ��λ��r/min	��ʽ�� REDUCTION_RATE * 60 / C
	MotorPulseWANZEToMotorSpeed = 1 / MotorSpeedZLACToPulse;	
	//�Ƕ����ֵ->�������ֵ����λ��0.1 degree -> radian
	Z_Radian_Max = pdu[max_angle] * MAGNIFIC_10x_DOWN * ANGtoRAD;	
}	

/**************************************************************************
�������ܣ����ˢ������
��ڲ������� 
����  ֵ����
**************************************************************************/
void Motor_task(void* pvParameters)
{
	while (1){
		rt_thread_delay(50);//��������5ms��Ƶ������
		Detect_Motor_Status();
		Set_MotorVelocity();
	}
}

/**********************************************************
 * �������ܣ� ����TPDO��������Ƿ�����
 * ˵    ���� ��TPDO���յ�����Ϣ����ڽṹ���С�
 *            ���������ߣ�����Ҫ���³�ʼ�������
 **********************************************************/
void Detect_Motor_Status(void)
{
	int i, j, k;
	uint16_t type, sport_mode, state_word, temperature, voltage, current, error_code,
			position1_feedback, position2_feedback, rpm_feedback, torgue_feedback;//�����н���
	uint16_t mpl1, mpl2, mpr1, mpr2, ms1, ms2;//����Ҫ�õ�����ʱ����
	int rpm_feedback_temp;//�����н���
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
								pdu[state_word]  = TPDOMessage[i][j]->Data[k++];					//״̬��
								pdu[state_word] |= TPDOMessage[i][j]->Data[k++] << 8;	
								pdu[position1_feedback]  = TPDOMessage[i][j]->Data[k++];			//��ǰλ�ã����ֽڣ�
								pdu[position1_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[position2_feedback]  = TPDOMessage[i][j]->Data[i++];			//��ǰλ�ã����ֽڣ�
								pdu[position2_feedback] |= TPDOMessage[i][j]->Data[i++] << 8;	
								pdu[temperature]  = TPDOMessage[i][j]->Data[k++];				//���PCB�¶�
								pdu[temperature] |= TPDOMessage[i][j]->Data[k++] << 8;
								break;
							case servo_zlacd:
								pdu[state_word]  = TPDOMessage[i][j]->Data[k++];					//״̬��
								pdu[state_word] |= TPDOMessage[i][j]->Data[k++] << 8;	
								pdu[motor1_rpm_feedback]  = TPDOMessage[i][j]->Data[k++];			//����
								pdu[motor1_rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[motor2_rpm_feedback]  = TPDOMessage[i][j]->Data[k++];			//�ҵ��
								pdu[motor2_rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								break;				
							case servo_plan:
								/* code */
								break;	
						}
					case TPDO1:
						switch (pdu[type]){
							case servo_zlac:
								pdu[rpm_feedback]  = TPDOMessage[i][j]->Data[k++];				//��ǰ�ٶ�
								pdu[rpm_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;
								k += 2;//��ȥk = 2, k = 3����
								switch (pdu[sport_mode]){
									case torque_mode:
										pdu[torgue_feedback]  = TPDOMessage[i][j]->Data[k++];		//��ǰת��
										pdu[torgue_feedback] |= TPDOMessage[i][j]->Data[k++] << 8;									
										break;
									default:
										pdu[voltage]  = TPDOMessage[i][j]->Data[k++];				//ĸ�ߵ�ѹ
										pdu[voltage] |= TPDOMessage[i][j]->Data[k++] << 8;	
										break;
								}
								pdu[error_code]  = TPDOMessage[i][j]->Data[k++];					//�������
								pdu[error_code] |= TPDOMessage[i][j]->Data[k++] << 8;
							case servo_wanze:
								rpm_feedback_temp  = TPDOMessage[i][j]->Data[k++];					//��ǰ�ٶȣ����壩
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 8;
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 16;			
								rpm_feedback_temp |= TPDOMessage[i][j]->Data[k++] << 24;
								rpm_feedback_temp = (int)(rpm_feedback_temp * MotorPulseWANZEToMotorSpeed);
								pdu[rpm_feedback]  = rpm_feedback_temp & 0xFFFF;					//��ǰ�ٶȣ�0.1rpm��		
								pdu[voltage]  = TPDOMessage[i][j]->Data[k++];						//ĸ�ߵ�ѹ
								pdu[voltage] |= TPDOMessage[i][j]->Data[k++] << 8;
								pdu[current]  = TPDOMessage[i][j]->Data[k++];						//ĸ�ߵ���
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
	for(int count = 0; count < pdu[motor_number]; count++){								//����none_count		
		if(mtd[count].d.none_count < 10000){
			mtd[count].d.none_count ++;
		}
		if(mtd[count].d.none_count > Offline_Time_1s || pdu[can_reinitialize] == 5){	//1s����PDO�ظ��������õ��CANͨ��
			mtd[count].d.online = false;												//�������״̬Ϊ����
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


	//����λ�Ʋ��ʽ��������/һȦ������/���ٱ�*��̥�ܳ�(m)*1000 = mm 	
	if(pdu[control_mode] == control_mode_ros){//����ģʽ��ROS
		NowPos_Left 	= (int)((uint32_t)mpl2 << 16 | mpl1);
		NowPos_Right 	= (int)((uint32_t)mpr2 << 16 | mpr1);		
		DiffposForOdom 	= (uint32_t)(round((float)((myabs(NowPos_Left - LastPos_Left) + myabs(NowPos_Right - LastPos_Right)) / 2) 
									/ pdu[motor2_reduction_ratio] / ENCODER_LINES * TireCircumference * MAGNIFIC_1000x_UP));//���λ:mm
		Odom_distance_mm += DiffposForOdom;
		pdu[Odom1ForRobot] = Odom_distance_mm & 0xFFFF;			//��̼Ƽ�¼
		pdu[Odom2ForRobot] = (Odom_distance_mm >> 16) & 0xFFFF;	//��̼Ƽ�¼
		LastPos_Left = NowPos_Left;
		LastPos_Right = NowPos_Right;
	}
	if((short)pdu[ms1] > 0){//���ݵ�����ٶȷ������ж����з���
		pdu[linear_speed_feedback]	= (short)(((short)ms1 + ((short)(0xFFFF - ms2 + 1) & 0xFFFF)) / 2 * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP);//���ٶȷ���(0.1r/min -> mm/s)
	}else{
		pdu[linear_speed_feedback]	= -(short)(((short)ms2 + ((short)(0xFFFF - ms1 + 1) & 0xFFFF)) / 2 * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP);//���ٶȷ���(0.1r/min -> m/s)
	}	
	pdu[yaw_speed_feedback] = -(short)(((float)(ms1 - ((0xFFFF - ms2 + 1) & 0xFFFF)) * MAGNIFIC_10x_DOWN * MotorSpeedToLineSpeed * MAGNIFIC_1000x_UP) / ((float)pdu[car_tread] * MAGNIFIC_10000x_DOWN));//���ٶȷ���
	// pdu[yaw_speed_feedback] = -(short)(((float)(pdu[motor2_rpm_feedback] - ((0xFFFF - pdu[motor3_rpm_feedback] + 1) & 0xFFFF)) / 10 * MotorSpeedToLineSpeed * 1000) / ((float)pdu[car_tread] / 10000));//���ٶȷ���
	//���ٶȹ�ʽ���󣬵ȴ��Ƕȿ���֮��������ȷ��ʽ������ٶ�
}

/**************************************************************************
�������ܣ����õ���ٶ�
��ڲ����������ֱ������Ƕ�Ӧ��can�ӻ�id�͵���ٶ�
�� �� ֵ����
**************************************************************************/
void Set_MotorVelocity(void)
{
	uint16_t Car_Error_Label = 0; //��������־
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
						last_nMotor[0] = mrd[0].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[0];//�������	
						last_nMotor[1] = mrd[1].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[1];//�������					
					}else{
						diff_nMotor[count] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[x_target_rpm] - last_nMotor[count]), VELOCITY_LIMIT));
						last_nMotor[count] = mrd[count].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[count]; //�������
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
						last_nMotor[0] = mrd[0].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[0];//�������	
						last_nMotor[1] = mrd[1].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[1];//�������					
					}else{
						diff_nMotor[count] = fmax(-VELOCITY_LIMIT, fmin(((short)pdu[x_target_torque] - last_nMotor[count]), VELOCITY_LIMIT));
						last_nMotor[count] = mrd[count].d.target_pos_vel += (short)pdu[x_direction] * diff_nMotor[count]; //�������
					}
					break;
			}
		}else{
			Car_Error_Label++; //ÿ����һ����������� ���ۼ�һ��
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
 * �������ܣ� ���ݽڵ�״̬�������ź��жϳ���״̬
 * ������     �������״̬��״̬����㡢�ֱ���ͣ��־λ
 * ����ֵ��   ���� CarRunningState ֵ��
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
 * �������ܣ� �����ʼ����
 * ������     IDΪ�ӻ���ַ��
 * ˵����     �ޡ�
 **********************************************************/
void Motor_Init(int count)
{		
	uint16_t i_start = (count == pdu[motor_number])? 0 : count;
	uint16_t i_end = (count == pdu[motor_number])? pdu[motor_number] : (count + 1);
	uint16_t mode, id, ah, al, dh, dl;
	for (int i = i_start; i < i_end; i++) {	//��ʼ��MOTOR����
		mode = pdu[motor1_sport_mode + i * pdu[ro_motor_gap]];
		id = pdu[motor1_CAN_id + i * pdu[rw_motor_gap]];
		ah = (pdu[motor1_acceleration_time + i * pdu[rw_motor_gap]] >> 8) & 0x00FF;
		al = pdu[motor1_acceleration_time + i * pdu[rw_motor_gap]] & 0x00FF;
		dh = (pdu[motor1_deceleration_time + i * pdu[rw_motor_gap]] >> 8) & 0x00FF;
		dl = pdu[motor1_deceleration_time + i * pdu[rw_motor_gap]] & 0x00FF;
		switch(mode){
			case servo_zlac:
				ZLAC8015_PDO_Config(id, mode, ah, al, dh, dl);//���PDO����			
				Driver_JT_Inv(id);			//��ת��ƽ	
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
		NMT_Control(Start_Command, id); 			//��������
	}			
}

/**********************************************************
 * �������ܣ� ͨ�õ��޷�������
 * ������     �ޡ�
 * ˵����     �ޡ�
 **********************************************************/
short limit_value(short value, short max_value) 
{
    return fmax(-max_value, fmin(value, max_value));
}

/**********************************************************
 * �������ܣ� can1�жϷ�����
 * ������     ��
 * ˵����     ͨ�����TPDO��Ϣ���жϴӻ��Ƿ�����
 **********************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN1);
}

/**********************************************************
 * �������ܣ� can2�жϷ�����
 * ������     ��
 * ˵����     ͨ�����TPDO��Ϣ���жϴӻ��Ƿ�����
 **********************************************************/
void CAN2_RX0_IRQHandler(void)
{
	CanIRQProcess(CAN2);
}

/**********************************************************
 * �������ܣ� can�жϷ�����
 * ������     ��
 * ˵����     ͨ�����TPDO��Ϣ���жϴӻ��Ƿ�����
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




