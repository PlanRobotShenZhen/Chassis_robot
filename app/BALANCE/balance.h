#ifndef __BALANCE_H
#define __BALANCE_H			

#include "stdint.h"
#include"stdbool.h"

#define BALANCE_TASK_PRIO 4	 //�������ȼ�
#define BALANCE_STK_SIZE 512 //�����ջ��С

#define VELOCITY_MULTI 3 //�ٶ�ֵ����

//���ٶ��������
#define AKM_RAW_FACTOR 1.277f

#define EPSILON 1e-6  // �ʵ��ļ�Сֵ
#define CORRECTION_FACTOR 1.35f  // ����ϵ��
#define ENCODER_LINES 10000  //һȦ������Ĭ��Ϊ10000
#define MYHALF 0.5f  //һ��(0.5)
#define RADtoANG 180.0f / PI	//����->�Ƕ�
#define ANGtoRAD PI / 180.0f	//�Ƕ�->����
#define DEGREE_MAX 300.0f 	//ǰ�����ת��


#define MAGNIFIC_10x_DOWN 		0.1f		//��С10��
#define MAGNIFIC_100x_DOWN 		0.01f		//��С100��
#define MAGNIFIC_1000x_DOWN 	0.001f		//��С1000��
#define MAGNIFIC_10000x_DOWN 	0.0001f		//��С10000��
#define MAGNIFIC_10x_UP 		10.0f		//����10��
#define MAGNIFIC_100x_UP 		100.0f		//����100��
#define MAGNIFIC_1000x_UP 		1000.0f		//����1000��
#define MAGNIFIC_10000x_UP 		10000.0f	//����10000��
#define MINTOSEC 				60.0f		//����->����

//SWB�ٶ�ģʽ�µ����ص�λ(VRA����)ϵ��
#define SWB_LOW_GEAR 			20			//���ص͵�λϵ��
#define SWB_MIDDLE_GEAR 		70			//�����е�λϵ��
#define SWB_HIGH_GEAR 			130			//���ظߵ�λϵ�� 

#define CHANNEL_VALUE_ERROR 	10			//�ֱ�ͨ��ֵ�������

#define TORQUE_COEFFICIENT_MAX 	3			//������ص�
#define TORQUE_COEFFICIENT_BASE 2			//�е����ص�
#define TORQUE_COEFFICIENT_MIN 	1			//��С���ص�

extern float Move_X,Move_Y,Move_Z;   	//С����������ٶ�
extern float Voltage;
extern float Z_Radian_Max; 				//�������ֵ
extern bool ROS_JT_Flag;

/*----------------------��С���ٶȺ궨��--------------------------*/
#define MAXDEGREE 300 		//ǰ�����ת��
//Akm_Car
#define AKMCAR_MAXLINEARVELOCITY 		1728									//������ٶ�
#define AKMCAR_MAXYAWVELOCITY 			1330									//�����ٶ�
#define AKMCAR_SPEED_LOW 				AKMCAR_MAXLINEARVELOCITY / 3			//���ٵ����ٶ�ϵ��
#define AKMCAR_SPEED_MIDDLE 			AKMCAR_MAXLINEARVELOCITY / 3 * 2		//���ٵ����ٶ�ϵ��
#define AKMCAR_SPEED_HIGH 				AKMCAR_MAXLINEARVELOCITY				//���ٵ����ٶ�ϵ��
#define AKMCAR_YAW_LOW 					AKMCAR_MAXYAWVELOCITY / 3				//���ٵ����ٶ�ϵ��
#define AKMCAR_YAW_MIDDLE 				AKMCAR_MAXYAWVELOCITY / 3 * 2			//���ٵ����ٶ�ϵ��
#define AKMCAR_YAW_HIGH 				AKMCAR_MAXYAWVELOCITY					//���ٵ����ٶ�ϵ��
//Diff_Car		
#define DIFFCAR_MAXLINEARVELOCITY 		800										//������ٶ�
#define DIFFCAR_MAXYAWVELOCITY 			1500									//�����ٶ�
#define DIFFCAR_SPEED_LOW 				DIFFCAR_MAXLINEARVELOCITY / 3			//���ٵ����ٶ�ϵ��
#define DIFFCAR_SPEED_MIDDLE 			DIFFCAR_MAXLINEARVELOCITY / 3 * 2		//���ٵ����ٶ�ϵ��
#define DIFFCAR_SPEED_HIGH 				DIFFCAR_MAXLINEARVELOCITY				//���ٵ����ٶ�ϵ��
#define DIFFCAR_YAW_LOW 				DIFFCAR_MAXYAWVELOCITY / 15 * 3			//���ٵ����ٶ�ϵ��
#define DIFFCAR_YAW_MIDDLE 				DIFFCAR_MAXYAWVELOCITY / 15 * 5 		//���ٵ����ٶ�ϵ��
#define DIFFCAR_YAW_HIGH 				DIFFCAR_MAXYAWVELOCITY / 15 * 8			//���ٵ����ٶ�ϵ��
//FourWheel_Car		
#define FOURWHEELCAR_MAXLINEARVELOCITY 	1500									//������ٶ�
#define FOURWHEELCAR_MAXYAWVELOCITY 	3000									//�����ٶ�
#define FOURWHEELCAR_SPEED_LOW 			FOURWHEELCAR_MAXLINEARVELOCITY / 3		//���ٵ����ٶ�ϵ��
#define FOURWHEELCAR_SPEED_MIDDLE 		FOURWHEELCAR_MAXLINEARVELOCITY / 3 * 2	//���ٵ����ٶ�ϵ��
#define FOURWHEELCAR_SPEED_HIGH 		FOURWHEELCAR_MAXLINEARVELOCITY			//���ٵ����ٶ�ϵ��
#define FOURWHEELCAR_YAW_LOW 			FOURWHEELCAR_MAXYAWVELOCITY / 3			//���ٵ����ٶ�ϵ��
#define FOURWHEELCAR_YAW_MIDDLE 		FOURWHEELCAR_MAXYAWVELOCITY / 3 * 2		//���ٵ����ٶ�ϵ��
#define FOURWHEELCAR_YAW_HIGH 			FOURWHEELCAR_MAXYAWVELOCITY				//���ٵ����ٶ�ϵ��
//TwoWheel_Car	
#define TWOWHEELCAR_MAXLINEARVELOCITY 	1000									//������ٶ�
#define TWOWHEELCAR_MAXYAWVELOCITY 		1500									//�����ٶ�
#define TWOWHEELCAR_SPEED_LOW 			TWOWHEELCAR_MAXLINEARVELOCITY / 3		//���ٵ����ٶ�ϵ��
#define TWOWHEELCAR_SPEED_MIDDLE 		TWOWHEELCAR_MAXLINEARVELOCITY / 3 * 2	//���ٵ����ٶ�ϵ��
#define TWOWHEELCAR_SPEED_HIGH 			TWOWHEELCAR_MAXLINEARVELOCITY			//���ٵ����ٶ�ϵ��
#define TWOWHEELCAR_YAW_LOW 			TWOWHEELCAR_MAXYAWVELOCITY / 3			//���ٵ����ٶ�ϵ��
#define TWOWHEELCAR_YAW_MIDDLE 			TWOWHEELCAR_MAXYAWVELOCITY / 3 * 2		//���ٵ����ٶ�ϵ��
#define TWOWHEELCAR_YAW_HIGH 			TWOWHEELCAR_MAXYAWVELOCITY				//���ٵ����ٶ�ϵ��
//Tank_Car	
#define TANKCAR_MAXLINEARVELOCITY 		1000									//������ٶ�
#define TANKCAR_MAXYAWVELOCITY 			1500									//�����ٶ�
#define TANKCAR_SPEED_LOW 				TANKCAR_MAXLINEARVELOCITY / 3			//���ٵ����ٶ�ϵ��
#define TANKCAR_SPEED_MIDDLE 			TANKCAR_MAXLINEARVELOCITY / 3 * 2		//���ٵ����ٶ�ϵ��
#define TANKCAR_SPEED_HIGH 				TANKCAR_MAXLINEARVELOCITY				//���ٵ����ٶ�ϵ��
#define TANKCAR_YAW_LOW 				TANKCAR_MAXYAWVELOCITY / 3				//���ٵ����ٶ�ϵ��
#define TANKCAR_YAW_MIDDLE 				TANKCAR_MAXYAWVELOCITY / 3 * 2			//���ٵ����ٶ�ϵ��
#define TANKCAR_YAW_HIGH 				TANKCAR_MAXYAWVELOCITY					//���ٵ����ٶ�ϵ��

/*----------------------���Ŀ��ƺ���--------------------------*/
void Ultrasonic1_task(void* pvParameters);
void Balance_task(void *pvParameters);           //������
unsigned char Turn_Off(void);                    //���ݵ�ص���������
void IrDA_TX_Control(void);
void IrDA_Guide(void);
void IrDA_Send0(void);
void IrDA_Send1(void);
void IrDA_SendData(uint8_t data);
void IrDA_RX_Decode(void);
void Relay_Switch(void);								// �̵������ؿ��ƺ��������ڿ��Ƽ̵����Ĵ򿪺͹ر�
void ChargerBalanceInit(void);
void SetReal_Velocity(void);                         //���õ�����е��ٶ�       
void ClassificationOfMotorMotionModes(uint16_t sport_mode);	   
void ServoPulse_Enable(void);             
void BatteryInformation(void);
void SPI1_ReadWriteByte(void);
void PowerControl(void);
/*----------------------�������ܺ���--------------------------*/
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

