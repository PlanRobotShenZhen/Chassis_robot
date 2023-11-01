#ifndef __can__h
#define __can__h
#include "n32g45x.h"                    // Device header


#define CAN_RX0_INT_ENABLE 1   //ʹ���жϱ�ʶ��


extern int g_nMotorLF_config;
extern int g_nMotorLB_config;  
extern int g_nMotorRF_config;
extern int g_nMotorRB_config;  //�������������ñ���


extern int g_nReInitMotor_LB;   //�����������ϵ��������δ�ϵ磬���³�ʼ��������������
extern int g_nReInitMotor_LF;   //Ϊ1������Ҫ���³�ʼ����Ϊ0�������Ҫ���³�ʼ��
extern int g_nReInitMotor_RF;
extern int g_nReInitMotor_RB;


extern int g_nHeart_count_LB;    //���������������ڹ涨������û��������������Ϊ�ӻ�����
extern int g_nHeart_count_LF;    //����״̬����ΪSTATE_STOP
extern int g_nHeart_count_RF;
extern int g_nHeart_count_RB;

extern int g_nHeart_Lastcount_LB;  //��¼�ϴν����������ļ���ֵ
extern int g_nHeart_Lastcount_LF;
extern int g_nHeart_Lastcount_RF;
extern int g_nHeart_Lastcount_RB;


//�����ŷ����ٶȵĹ�����
union URcv_Vel_Data{
		int nVelcity;
		unsigned char ucData[4];
};

// �������������Ͻ��յĹ�����
union URcv_ERROR_Data{
		unsigned short usError;
		unsigned char ucData[2];
};


//����������״̬
enum ENUM_HEART_STAT
{
		STATE_UNKNOW      = 0x00,   //δ֪״̬
		STATE_STOP        = 0x04,		//ֹͣ״̬
		STATE_START       = 0x05,		//����״̬
		STATE_PRE_OPERATE = 0x7F		//Ԥ����״̬
};

// �����������Ĺ���״̬
enum ENUM_ERROR_STATE
{
	ERROR_NONE              = 0x00,  // �޴���
	ERROR_OVER_VALUE        = 0x01,  // ��ѹ
	ERROR_LESS_VALUE        = 0x02,  // Ƿѹ
	ERROR_OVER_CURRENT      = 0x04,  // ����
	ERROR_OVER_LOAD         = 0x08,  // ����
	ERROR_OVERDIFF_CURRENT  = 0x10,  // ��������
	ERROR_OVERDIFF_ENCODER  = 0x20,  // ����������
	ERROR_OVERDIFF_VELOCITY = 0x40,  // �ٶȳ���
	ERROR_REF_VALUE         = 0x80,  // �ο���ѹ����
	ERROR_EEPROM            = 0x100, // EEPROM��д����
	ERROR_HALL              = 0x200  // ��������
};

extern union URcv_Vel_Data uVelLF;
extern union URcv_Vel_Data uVelLB;
extern union URcv_Vel_Data uVelRF;
extern union URcv_Vel_Data uVelRB;  //�������ʵʱ�ٶȵı���

extern union URcv_ERROR_Data uError; // �����������Ĺ���״̬

extern enum ENUM_HEART_STAT eLF_Motor_Heart;
extern enum ENUM_HEART_STAT eLB_Motor_Heart;
extern enum ENUM_HEART_STAT eRF_Motor_Heart; 
extern enum ENUM_HEART_STAT eRB_Motor_Heart;   //���ҵ��������

extern enum ENUM_ERROR_STATE eLF_Motor_Error;
extern enum ENUM_ERROR_STATE eLB_Motor_Error;
extern enum ENUM_ERROR_STATE eRF_Motor_Error;
extern enum ENUM_ERROR_STATE eRB_Motor_Error;   // �������������ȡ

void Can_Driver_Init(void);

#endif

