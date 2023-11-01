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
float VelocityToRpmConversion;//�ٶȻ�����ת��
float AngularVelocityConversion;//���ٶȻ���



/**************************************************************************
�������ܣ��Խ��յ����ݽ��д���
��ڲ�����X��Y Z�᷽����˶��ٶ�
����  ֵ����
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
	int i = car_max_lin_speed; //��ʼ����
	//С�����ٶȺͽ��ٶ��޷�
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
	//����С����Ŀ�����ٶȺͽ��ٶ�
	tmp.v = Move_X;
	i = 158;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = Move_Z;
	pdu[i++] = tmp.ud[1];
	pdu[i] = tmp.ud[0];
	//������ ---- ��ǰ���õ��ĳ�
	if(g_emCarMode == FourWheel_Car) 
	{	
		//�õ�ʱ��motora��motorbҪ���õ�Ϊ����
		float tmp_value = Vz * (Wheel_spacing +  Axle_spacing) / 2.0f;
		//MOTOR_A.fltTarget_velocity  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //���������ֵ�Ŀ���ٶ�
		//MOTOR_B.fltTarget_velocity = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //�������ǰ�ֵ�Ŀ���ٶ�
		//MOTOR_C.fltTarget_velocity  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //�������ǰ�ֵ�Ŀ���ٶ�
		//MOTOR_D.fltTarget_velocity  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //������Һ��ֵ�Ŀ���ٶ�
		MOTOR_A.fltTarget_velocity = Vx + tmp_value;
		MOTOR_B.fltTarget_velocity = MOTOR_A.fltTarget_velocity; //�������ǰ�ֵ�Ŀ���ٶ�
		MOTOR_C.fltTarget_velocity = Vx - tmp_value;
		MOTOR_D.fltTarget_velocity = MOTOR_C.fltTarget_velocity;
		//�����ĸ������Ŀ���ٶ�
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

		int tva = SpeedVelocityToRotate(MOTOR_A.fltTarget_velocity); //ת��Ϊr/min
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
�������ܣ�ͨ����ģң�ضԻ����˽���ң��,ֻ��Ҫx���ٶ���z����ٶ�
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control()
{
	//�Ժ�ģ����������ֵת�������ٶȺͽ��ٶ�
	//Move_X = ((float)g_nVelocity * 2 * PI * pdu[61] / 100) / 60 / pdu[62];    //��λΪm/s,����pdu[61]Ҫ����10000��pdu[62]Ҫ����100����ֻ�����100
	Move_X = (float)g_nVelocity * FourWheer_Conversion;    //��λΪm/s
	Move_Y = 0;
	Move_Z = ((float)g_nDirector) / AngularVelocityConversion;

}	



/**************************************************************************
�������ܣ�ͨ����λ���Ի����˽���ң��,ֻ��Ҫx���ٶ���z����ٶ�
��ڲ�������
����  ֵ����
**************************************************************************/
void Ros_Control()
{
	Move_X = g_fltRecv_Vel_X;
	Move_Y = g_fltRecv_Vel_Y;
	Move_Z = -g_fltRecv_Vel_Z;    // ��Ҫ����Z�����ݣ������з��ֵ�
}	


/**************************************************************************
�������ܣ���ȡ4������ٶȡ�����can�жϣ���ȡ�õ�4��������ٶȣ�
					����ڶ�Ӧ��Motor�ṹ���С�r/min
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Get_Motor_Velocity()
{
	//����ɼ����������������������������ݣ�Ҫ������Ӧת������
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
�������ܣ��������ת�٣�r/minת�����������ٶ� m/s��
					������ٶȽṹ���С�
					1 r/min = ��2�� * R�� / 60  m/s
��ڲ�����nRotateSpeed����ת�٣�r/min
�� �� ֵ������ת��
**************************************************************************/
float RotateToSpeedVelocity(int nRotateSpeed)
{
	//float fltReturn = 0.0f;
	//v=n*2*pi*r/60
	//fltReturn = nRotateSpeed * 2 * PI * FourWheer_Radiaus/ 60;  //ת��Ϊ��̥�ٶ�
	// 2 * PI * FourWheer_Radiaus/ 60 = 0.0172787593
	//fltReturn = nRotateSpeed * 0.0172787593;  //ת��Ϊ��̥�ٶ�
	return nRotateSpeed * FourWheer_Perimeter/60;  //ת��Ϊ��̥�ٶ�;	
}


/**************************************************************************
�������ܣ������ӵ��ٶȣ�m/sת���ɵ����ת�� r/min��
					������ٶȽṹ���С�
					1 m/s = 60 /��2 * PI * R��
��ڲ�����fltSpeed��������ת�٣�m/s
�� �� ֵ��������ת��
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
�������ܣ��������˵�Ŀ���ٶ���ƽ�����ƴ���ֻ�������ķ��С����Ҫ
��ڲ���������������Ŀ���ٶ�
����  ֵ����
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
�������ܣ�������ʱ�������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Update_Gyroscope(void)
{	
	memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));//˫���������������
}


/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
unsigned char Turn_Off( )
{
	unsigned char ucTemp;
	if((g_nVol_get_Flag && g_fltProprity_Voltage < 0.2) || Flag_Stop == 1)//��ص�ѹ���͹رյ��
	{	 
		//���õ��Ŀ���ٶ�Ϊ0����ʧ�ܵ��
		ucTemp = 1;      
		Set_MotorVelocity(0, 0, 0, 0);   //����Ŀ�������ٶ�
	}
	else if(g_nVol_get_Flag && g_fltProprity_Voltage >= 0.2)
	{
		ucTemp = 0;
	}
	return ucTemp;			
}


/**************************************************************************
�������ܣ�����4������ٶ�
��ڲ����������ֱ������������ǰ����ǰ���Һ󷽶�Ӧ��can�ӻ�id�͵���ٶ�
�� �� ֵ����
**************************************************************************/
void Set_MotorVelocity(int nMotorLB,int nMotorLF, int nMotorRF, int nMotorRB)
{//Is_Offline();target_velocity
	mrd[0].d.target_velocity = mrd[0].d.online ? nMotorLB : 0;
	mrd[1].d.target_velocity = mrd[1].d.online ? nMotorLF : 0;
	mrd[2].d.target_velocity = mrd[2].d.online ? nMotorRB : 0;
	mrd[3].d.target_velocity = mrd[3].d.online ? nMotorRF : 0;
}

/*---------------------------һЩ���ܺ���--------------------------------*/
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}



/**************************************************************************
�������ܣ�����������ȡ����ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


/**************************************************************************
�������ܣ��޷��������趨�ߵ���ֵ
��ڲ�������ֵ
����  ֵ��
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
�������ܣ����Ŀ������
��ڲ�����
����  ֵ��
**************************************************************************/
void Balance_task(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt(); //��ȡ��ϵͳ�ϴ�ִ��ʱ��
	pdu = getPDUData();
	// pdu[61] ���1���ְ뾶 pdu[62] ���1���ٱ�
	//FourWheer_Perimeter = 2 * PI * FourWheer_Radiaus;
	union {
		float v;
		int16_t ud[2];
	}tmp;
	int i = car_max_lin_speed; //��ʼ����
	tmp.v = 0.802;//< ������������ٶȳ�ʼ��
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.802;//< ��������С���ٶ�
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = 0.8014;//< �����������ٶ�
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.8014;//< ��������С���ٶ�
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];


	pdu[motor1_radius] = FourWheer_Radiaus * 10000;
	pdu[motor1_reduction_ratio] = REDUCTION_RATE * 100;

	float Radiaus = pdu[motor1_radius] * 0.0001;
	float rate = pdu[motor1_reduction_ratio] * 0.01;
	FourWheer_Perimeter = 2 * PI * Radiaus;//< �����ܳ�
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
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); //��������100Hz��Ƶ�����У�10ms����һ�Σ�

		g_eControl_Mode = pdu[car_mode]&0xff;
		Get_Motor_Velocity();                  //��ȡ�������������ٶȣ�����ڽṹ����
		SetReal_Velocity(pdu);                  //������ģ����
		//ʹ����λ�����Ƶ�ʱ�򣬻���usart�ж��йر�Remote_ON_Flag��־λ
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
				Remote_Control();                    //��ģ��ȡ���趨���ٶ�
			}
			else if (g_eControl_Mode == CONTROL_MODE_ROS)
			{
				// ��λ�����Ƶ�ʱ�򣬴�ʱZ����ٶ�Ҫ����һ�����򣬲�Ȼ����ת���ʱ��������
				Ros_Control();
			}
			motorA_ptr->motor_state = 1;
			motorB_ptr->motor_state = 1;
			motorC_ptr->motor_state = 1;
			motorD_ptr->motor_state = 1;
			
		}
		pdu[CONTROL_MODE_ADDR] = g_eControl_Mode;//
		Drive_Motor(Move_X, Move_Y, Move_Z);   //С���˶�ģ�ͽ�����ÿ�������ʵ���ٶ�

		switch (g_emCarMode)
		{
		case Mec_Car:        break; //�����ķ��С��
		case Omni_Car:       break; //ȫ����С��
		case Akm_Car:        break; //������С��
		case Diff_Car:       break; //���ֲ���С��
		case FourWheel_Car:
			Set_MotorVelocity(-MOTOR_A.nTarget_Velocity, -MOTOR_B.nTarget_Velocity,
				MOTOR_C.nTarget_Velocity, MOTOR_D.nTarget_Velocity);

			break; //������ 
		case Tank_Car:
			break; //�Ĵ���

		default: break;
		}
		
		pdu[car_mode] = (pdu[car_mode] & 0xFF00) | g_eControl_Mode;
		pdu[error_get_and_clear] = error_code;//< ���ϻ�ȡ
		pdu[car_error_messages] = error_code;
		if (error_code != ERROR_NONE)
		{
			pdu[car_system_state] = 2;
		}
	}
}
