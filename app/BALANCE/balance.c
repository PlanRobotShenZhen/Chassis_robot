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
float VelocityToRpmConversion;//�ٶȻ�����ת��
float AngularVelocityConversion;//���ٶȻ���
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

/*����ͨѶ���Ͳ��ֲ���*/
uint8_t IrDA_SendState = 0;		//0���رպ��⴫������1������Խӣ�2������ͨѶ��
uint8_t SendCout = 0;			//���ͼƴ�
uint8_t SendGuide_Flag = 0;		//����λ״̬
int Send_i = 3;					//��������ָ�룬�Ӹ�λ��ʼ����
int BitFree = 1;				//����ͨ������
uint8_t SendData = 0x01;		//Ҫ���͵�����
/*����ͨѶ���ղ��ֲ���*/
uint8_t ReceiveData;			//���յ�������
int LimitSwitch_OK = 0;		//��λ���أ��Ͽ�0���պ�1��

struct {
	unsigned char SW : 2;//�� ��ͣ����
	unsigned char Rising : 1;
	unsigned char Descending : 1;

	unsigned char estop_soft : 1;//< ��ͣ
	unsigned char estop_soft_old : 1;//< ��ͣ
}emergency_stop;

static uint8_t battery_send_frame_num = 0;
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
	int i;
	//������ ---- ��ǰ���õ��ĳ�
	if(g_emCarMode == FourWheel_Car) 
	{	//�õ�ʱ��motora��motorbҪ���õ�Ϊ����		
		float tmp_value = Vz * (Wheel_spacing +  Axle_spacing) / 2.0f;
		MOTOR_A.fltTarget_velocity = Vx + tmp_value;
		MOTOR_B.fltTarget_velocity = MOTOR_A.fltTarget_velocity; //�������ǰ�ֵ�Ŀ���ٶ�
		MOTOR_C.fltTarget_velocity = Vx - tmp_value;
		MOTOR_D.fltTarget_velocity = MOTOR_C.fltTarget_velocity;

		int tva = SpeedVelocityToRotate(MOTOR_A.fltTarget_velocity); //ת��Ϊr/min
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
		//	{//< ��ͣ
		//		acc = 50000;
		//	}
		//	else acc = pdu[robot_acceleration];
		//}
		if (exio_input.bit.X0 || emergency_stop.estop_soft)
		{//< ��ͣ
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
	{//< ����С��

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
	if (pdu[car_model] == RC_Car)
	{
		pdu[car_setting_lin_speed] = (int16_t)(Move_X * 1000);
		pdu[car_setting_ang_speed] = (int16_t)(Move_Z * 1000);
	}
	else if (pdu[car_model] == FourWheel_Car)
	{//< �������
		uint32_t vel_;
		pdu[car_setting_lin_speed] = (int16_t)(-Move_X * 1000);
		pdu[car_setting_ang_speed] = (int16_t)(-Move_Z * 1000);
		float nMotor_A = mtd[0].d.current_velocity;
		float nMotor_B = mtd[1].d.current_velocity;
		float nMotor_C = mtd[2].d.current_velocity;
		float nMotor_D = mtd[3].d.current_velocity;
		if (pdu[motor1_model] == SERVO_WANZE||pdu[motor1_model] == SERVO_PLAN)nMotor_A = nMotor_A * 60 / 10000;//< һȦ10000������
		if (pdu[motor2_model] == SERVO_WANZE||pdu[motor2_model] == SERVO_PLAN)nMotor_B = nMotor_B * 60 / 10000;//< һȦ10000������
		if (pdu[motor3_model] == SERVO_WANZE||pdu[motor3_model] == SERVO_PLAN)nMotor_C = nMotor_C * 60 / 10000;//< һȦ10000������
		if (pdu[motor4_model] == SERVO_WANZE||pdu[motor4_model] == SERVO_PLAN)nMotor_D = nMotor_D * 60 / 10000;//< һȦ10000������
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
�������ܣ��������ת�٣�r/minת�����������ٶ� m/s��
					������ٶȽṹ���С�
					1 r/min = ��2�� * R�� / 60  m/s
��ڲ�����nRotateSpeed����ת�٣�r/min
�� �� ֵ���������ٶ� m/s��
**************************************************************************/
float RotateToSpeedVelocity(float nRotateSpeed)
{
	//float fltReturn = 0.0f;
	//v=n*2*pi*r/60
	//fltReturn = nRotateSpeed * 2 * PI * FourWheer_Radiaus/ 60;  //ת��Ϊ��̥�ٶ�
	// 2 * PI * FourWheer_Radiaus/ 60 = 0.0172787593
	//fltReturn = nRotateSpeed * 0.0172787593;  //ת��Ϊ��̥�ٶ�
	return nRotateSpeed * FourWheer_Conversion;  //ת��Ϊ��̥�ٶ�;	
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
	if(pdu[motor1_direction]==1)mrd[0].d.target_velocity = -mrd[0].d.target_velocity;
	if(pdu[motor2_direction]==1)mrd[1].d.target_velocity = -mrd[1].d.target_velocity;
	if(pdu[motor3_direction]==1)mrd[2].d.target_velocity = -mrd[2].d.target_velocity;
	if(pdu[motor4_direction]==1)mrd[3].d.target_velocity = -mrd[3].d.target_velocity;
}

/*---------------------------һЩ���ܺ���--------------------------------*/
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{ 		   
	  uint32_t temp;
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

void PowerControl(void)
{
	static union {
		struct {
			uint16_t jdq1 : 1;//< ������Դ JDQ1_EN
			uint16_t jdq2 : 1;//< �ҵ����Դ JDQ2_EN
			uint16_t p12v : 1;//< 12V��Դ YL_7
			uint16_t p19v : 1;//< 19V��Դ YL_6

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
	int i = 0; //��ʼ����
	float MOVE_XorZ = 0;
	float temp;
	if (uart4_recv_flag)
	{
		uart4_recv_flag = 0;
		switch (pdu[battery_manufacturer])
		{
		case 1://< ���������Ƽ����޹�˾
			//С�����ٶȺͽ��ٶȱȽ�
			if (fabsf(Move_X) > fabsf(Move_Z))
			{
				MOVE_XorZ = Move_X;
				temp = ((int16_t)pdu[car_max_lin_speed]) * 0.001;
			} //��ʼ����
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
				pdu[BatteryStatus] = 1;//< ��ض�ȡ�ɹ�
				pdu[BatteryVoltage] = uart4_recv_data[6] | (uart4_recv_data[7] << 8);//< ��ص�ѹ
				pdu[BatteryCurrent] = uart4_recv_data[8] | (uart4_recv_data[9] << 8);//< ��ص���
				pdu[BatteryQuantity] = (uart4_recv_data[10] | (uart4_recv_data[11] << 8)) * 2;//< ��ص���
				pdu[BatteryHealth] = uart4_recv_data[12] | (uart4_recv_data[13] << 8);//< ��ؽ�����
				pdu[BatteryTemperature] = (uart4_recv_data[24] | (uart4_recv_data[25] << 8)) - 2731;//< ����¶�
				pdu[BatteryProtectStatus] = uart4_recv_data[36] | (uart4_recv_data[37] << 8);//< ��ر���״̬
			}
			break;
		default:
			bt_times = 100;//< 1000msˢ��1��
			if (uart4_recv_data[0] == 0x7e)
			{
				uint32_t tmp = 0;
				//uint16_t r_crc = (uint16_t)uart4_recv_data[uart4_recv_len - 2] | (((uint16_t)uart4_recv_data[uart4_recv_len - 1])<<8);
				//uint16_t c_crc = usMBCRC16(uart4_recv_data, uart4_recv_len - 2);			
				//if (r_crc == c_crc)
				// {
					int i = 0;
					pdu[BatteryStatus] = 1;//< ��ض�ȡ�ɹ�
					pdu[BatteryQuantity] = (uint16_t)uart4_recv_data[107] | (((uint16_t)uart4_recv_data[108]) << 8);//< ��ص���
					pdu[BatteryQuantity] *= 10;
					i = 97;
					tmp = (uint32_t)uart4_recv_data[i++];
					tmp |= (uint32_t)uart4_recv_data[i++] << 8;
					tmp |= (uint32_t)uart4_recv_data[i++] << 16;
					tmp |= (uint32_t)uart4_recv_data[i++] << 24;
					pdu[BatteryVoltage] = tmp*0.1;//< ��ص�ѹ
					tmp = (uint32_t)uart4_recv_data[i++];
					tmp |= (uint32_t)uart4_recv_data[i++] << 8;
					tmp |= (uint32_t)uart4_recv_data[i++] << 16;
					tmp |= (uint32_t)uart4_recv_data[i++] << 24;
					pdu[BatteryCurrent] = (int16_t)((int)tmp*0.1);//< ��ص���
					pdu[BatteryTemperature] = (uint16_t)uart4_recv_data[77] | (((uint16_t)uart4_recv_data[78]) << 8);//< ����¶�
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
		DMA_EnableChannel(USARTb_Tx_DMA_Channel, DISABLE);    // �ر� DMA2 ͨ��5, UART4_TX
		DMA_SetCurrDataCounter(USARTb_Tx_DMA_Channel, battery_send_frame_num);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
		DMA_EnableChannel(USARTb_Tx_DMA_Channel, ENABLE);    // ���� DMA2 ͨ��5, UART4_TX	
	}
}
//< ���������
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
//SPI��д����
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
		while ((SPI_MASTER->STS & 1 << 1) == 0) //�ȴ���������
		{
			retry++;
			if (retry > 2000)
			{
				error_code = 11;//< SPI����
				return;
			}
		}

		if (robot_control.bit.light_ctrl_en == 0)
		{//< Ĭ�ϵƿ���Ȩ
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
* �������ܣ��������ݽ���������ʹ��VRA�����ƣ���ȡͨ��10����ȡ����
* ����ֵ��
****************************************************************/

void Car_Light_Control(float Vx, float Vz)
{
	unsigned short usTemp = 0;
	usTemp = tagSBUS_CH.CH10;

	// �߼��ж�ģ��
	if (Abs_int(usTemp - rc_ptr->light_base) > 100)
	{// ��ʱ����û�д��������ƹ�ı�־		
		if ((usTemp - rc_ptr->light_base) > 0)
		{// �����ƹ�			
			g_ucLightOnFlag = 1;
		}
		else
		{// �رյƹ�			
			g_ucLightOnFlag = 0;
		}
	}
	


	usTemp = tagSBUS_CH.CH9;//< ģ����ͣ
	if (Abs_int(usTemp - 1023) > 100)
	{
		if ((usTemp - 1023) > 0)
		{// ������ͣ			
			emergency_stop.estop_soft = 1;			
		}
		else
		{// �ر���ͣ	
			emergency_stop.estop_soft = 0;			
		}
		if (emergency_stop.estop_soft_old != emergency_stop.estop_soft)
		{
			emergency_stop.estop_soft_old = emergency_stop.estop_soft;
			if (emergency_stop.estop_soft)
			{
				emergency_stop.Descending = 1;
				GPIO_ResetBits(RJ_JT_GPIO, RJ_JT_Pin);//< ������ͣ	
			}
			else GPIO_SetBits(RJ_JT_GPIO, RJ_JT_Pin);//< �ر���ͣ	
		}
	}

	if (g_eControl_Mode == CONTROL_MODE_UNKNOW)
	{//< �ȴ�����״̬
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
		{//< ����
			exio_output.output |= 0xa0;
		}
		if (Vz >0)
		{//< ��ת
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
		{//< ��ת
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
	case 1://< ���������Ƽ����޹�˾
		uart4_send_data[battery_send_frame_num++] = 0x3A;//< ֡ͷ
		uart4_send_data[battery_send_frame_num++] = 0x7E;//< ͨ�ã�0x7E
		uart4_send_data[battery_send_frame_num++] = 0x01;//< Э��汾��
		uart4_send_data[battery_send_frame_num++] = 0x01;//< 0��д 1����
		uart4_send_data[battery_send_frame_num++] = 0x1E;//< ������ (0x1E)
		uart4_send_data[battery_send_frame_num++] = 0x00;//< ����
		uart4_send_data[battery_send_frame_num++] = 0xD8;//< У���
		battery_send_frame_num = 9;
		break;
	default://7E 0A 01 00 00 30 00 AC 00 00 2C 90 //< �����Ϣ��ʼ����ȡ
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
�������ܣ�MCU_INF_RX���յ�����С���ĺ����ź�֮�󣬿������׮�ĺ��ⷢ�͹���
��ڲ�������
������Ϣ��δ���յ��ź�ʱ��RX�źŶ˵ͣ�ADCJT(PA0)�ߣ�
**************************************************************************/
void Sensor_TX_Control()
{
	//���յ������ź�
	if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO, MCU_INF_RX_PIN) == RESET)
	{
		//1�������⴫�������Ͷ�
		MCU_INF_TX = 1;
	}
	else
	{
		MCU_INF_TX = 0;
	}
}
/*******************  *******************************************************
�������ܣ�����ͨѶ�������ݲ���
��ڲ�������/Ҫ���͵�����
������Ϣ�������źŶ˵͵�ƽʱ��TX�źŶ˸ߣ�RGBG�ͣ�
**************************************************************************/

void IrDA_Guide(void)
{
	SendCout++;
	//�ߵ�ƽ100ms
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
	//�ߵ�ƽ30ms
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
	//�ߵ�ƽ70ms
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
	if (SendGuide_Flag == 0)//0:δ��������λ��1���ѷ�������λ��
	{
		IrDA_Guide();
	}
	else
	{
		uint8_t bit = (SendData >> Send_i) & 0x01;
		if (bit)
		{
			IrDA_Send1();
			if (BitFree)//�ȴ��������
			{
				Send_i--;//ָ��ָ���λ				
			}
		}
		else
		{
			IrDA_Send0();
			if (BitFree)//�ȴ��������
			{
				Send_i--;//ָ��ָ���λ				
			}
		}
		if (Send_i == -1)
		{
			//���ݷ������
			Send_i = 3;//ָ�����õ����ݸ�λ
			SendGuide_Flag = 0;//��������λ
		}
	}
}

void Relay_Switch(void)
{
	if (GPIO_ReadInputDataBit(MCU_CH_DET_GPIO, MCU_CH_DET_PIN) == RESET)
	{
		//�����̵���
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
�������ܣ����Ŀ������
��ڲ�����
����  ֵ��
**************************************************************************/

void Balance_task(void* pvParameters)
{
	uint8_t tmp1 = 0;
	pdu = (uint16_t*)pvParameters;
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
	BatteryInfoInit();
	if (pdu[robot_acceleration] < 5000)pdu[robot_acceleration] = 5000;
	if (pdu[car_model] == Charger)
	{
		/*���ⷢ�Ͷ˳�ʼ���ر�*/
		IrDA_TX = 0;
		/*���ȳ�ʼ������*/
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


		Get_Motor_Velocity();                  //��ȡ�������������ٶȣ�����ڽṹ����
		SetReal_Velocity(pdu);                  //������ģ����
		//ʹ����λ�����Ƶ�ʱ�򣬻���usart�ж��йر�Remote_ON_Flag��־λ
		if (g_eControl_Mode == CONTROL_MODE_UNKNOW)
		{
			Move_X = Move_Y = Move_Z = 0;
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
			//С�����ٶȺͽ��ٶ��޷�
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
			break; //�����ķ��С��
		case Omni_Car:       
			break; //ȫ����С��
		case Akm_Car:        
			break; //������С��
		case Diff_Car:       
			break; //���ֲ���С��
		case FourWheel_Car://������  �������
		{
			if (exio_input.bit.X0 || emergency_stop.estop_soft)
			{//< ��ͣ
				Move_X = Move_Y = Move_Z = 0;
			}
			if (pdu[robot_forward_direction] == 1)Move_X = -Move_X;
			if (pdu[robot_turning_direction] == 1)Move_Z = -Move_Z;
			Drive_Motor(Move_X, Move_Y, Move_Z);   //С���˶�ģ�ͽ�����ÿ�������ʵ���ٶ�
			Car_Light_Control(Move_X, Move_Z);
			Set_MotorVelocity(-MOTOR_A.nTarget_Velocity, -MOTOR_B.nTarget_Velocity,
				MOTOR_C.nTarget_Velocity, MOTOR_D.nTarget_Velocity);
		}
			break; 
		case Tank_Car:
			break; //�Ĵ���
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
			/*���ⷢ�Ͷ˹����л�*/
			switch (IrDA_SendState)
			{//0���رշ��Ͷˣ�1������Խӣ�2������ͨѶ��
			case 0:
				break;
			case 1:
				Sensor_TX_Control();
				//�Խӳɹ���תͨѶ	
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
			/*������ն˽���*/
			switch (ReceiveData)
			{
			case 0:
				break;
			case 0x01://����Խ�����
				SendData = 1;
				IrDA_SendData(SendData);
				//�ȴ���λ�����ź�
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
			case 0x02://�رճ��
				GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
				MCU_RELAY2 = 0;
				SendData = 2;
				IrDA_SendData(SendData);
				break;
			case 0x03://����쳣
				GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
				MCU_RELAY2 = 0;
				SendData = 3;
				IrDA_SendData(SendData);
				break;
			default:
				break;
			}
			//�����л�RGB��ɫ
			Key_Change_RGB();
			Relay_Switch();
		}
		break;
		default: break;
		}
		
		pdu[car_mode] = (pdu[car_mode] & 0xFF00) | g_eControl_Mode;
		pdu[error_get_and_clear] = error_code;//< ���ϻ�ȡ
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
