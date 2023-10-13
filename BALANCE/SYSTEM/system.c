#include "can.h"
#include "user_can.h"
#include "system.h"
#include "485_address.h"

float Wheel_perimeter = 0;    //轮子周长（单位：米）
float Wheel_spacing = 0;      //主动轮轮距 （单位：米）//后面会在robot_select_init.h文件中初始化
float Axle_spacing = 0;       //前后轴距
float Omni_turn_radiaus = 0;  //全向轮转弯半径

Remote_Control_struct *rc_ptr;
Motor_struct *motorA_ptr;
Motor_struct *motorB_ptr;
Motor_struct *motorC_ptr;
Motor_struct *motorD_ptr;
//Robot_speed *robot_speed_ptr;
enum CarMode g_emCarMode = UNKNOW;                                                //小车类型
unsigned char Flag_Stop = 0;                                                      //1代表使能机器人
enum ENUM_CarControl_Mode g_eControl_Mode = CONTROL_MODE_UNKNOW;  // 机器人的控制方式
unsigned char g_ucRemote_Flag = 0;              //航模开启标志位
unsigned char g_ucRos_Flag = 0;                 // ROS上位机进入标志位 

struct Motor_parameter  MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;

/****************************
* 函数功能：配置时钟源使用外部高速时钟HSE
* unDiv：RCC_PLLSource_HSE_Div1/RCC_PLLSource_HSE_Div2/RCC_PLLSource_HSI_Div2
*        此参数的功能是设计时钟源的来源。
* unPllm：设置的是倍频系数，RCC_PLLMul_2 ~ RCC_PLLMul_16
* 
****************************/
void RCC_HSE_Config(unsigned int unDiv, unsigned int unPllm)
{
	 RCC_DeInit();//将外部RCC寄存器重置为缺省值
	 RCC_HSEConfig(RCC_HSE_ON); //设置外部高速晶振(HSE)
	 if(RCC_WaitForHSEStartUp() == SUCCESS) //等待HSE起振
	 {
			RCC_HCLKConfig(RCC_SYSCLK_Div1); //设置AHB(HCLK)时钟
			RCC_PCLK1Config(RCC_HCLK_Div2); //设置APB1低速时钟(PCLK1)
			RCC_PCLK2Config(RCC_HCLK_Div1); //设置APB2高速时钟(PCLK2)
		 //配置PLL时钟源(u32Div)以及其倍频系数(u32Pill),
		 //来自外部高速时钟的几分频,第二个参数为PLL输出时钟及倍频,
		 //默认是9,即为72MHz?
			RCC_PLLConfig(unDiv,unPllm);
			RCC_PLLCmd(ENABLE);//使能或者失能PLL
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //检查指定的RCC标志位设置与否, PLL就绪
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //设置系统时钟(SYSCLK)
			while(RCC_GetSYSCLKSource()!=0x08);//返回用作系统时钟的时钟源,0x08:PLL作为系统时钟
	 }
}



// 手动软件复位
void Soft_Reset()
{
	int nDelay = 0;
	__DSB();
	while(nDelay < 100)
	{
		nDelay++;
	}
}
	

void systemInit(void)
{
	int nTime = 0;
	int i = turn_off_remote;
	uint16_t *pdu = getPDUData();
	while(nTime < 2000)
	{
		nTime++;
	}	
	// 设置系统时钟为72MHz，使用外部高速晶振HSE
	RCC_HSE_Config(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	
	delay_init();					          //初始化延时函数
	LED_Init();                     //初始化与LED连接的硬件接口
	USART1_Init(1500000);	        //=====串口初始化为，普通的串口，打印调试信息 DMA
	Usart3_init(115200);            //上下位机通信初始化，串口3
	Usart5_Init(100000);            //串口5初始化，用于航模控制
	//Adc_Init();                     //采集电池电压ADC引脚初始化	
	Can_Driver_Init();              //底层can协议初始化
	
	Robot_Select();                 // 根据电位器的值判断目前正在运行的是哪一款机器人，
	                                // 然后进行对应的参数初始化
	modbus_task_init();
	pdu[car_type] = 4;
	pdu[car_version] = 0x88;
	//初始化航模参数
	rc_ptr = (Remote_Control_struct*) &pdu[i];

	pdu[i++] = TURN_OFF_REMOTE;
	pdu[i++] = TURN_ON_REMOTE;
	pdu[i++] = VEL_BASE_VALUE;
	pdu[i++] = DIR_BASE_VALUE;
	pdu[i++] = LIMIT_MAX_VAL;
	pdu[i++] = LIMIT_MIN_VAL;
	pdu[i++] = SPEED_LEVEL1;
	pdu[i++] = SPEED_LEVEL2;
	pdu[i++] = SPEED_LEVEL3;
	pdu[i++] = SPEED_LOW;
	pdu[i++] = SPEED_MIDDLE;
	pdu[i++] = SPEED_HIGH;
	pdu[i++] = SPEED_DIR_LOW;
	pdu[i++] = SPEED_DIR_MIDDLE;
	pdu[i++] = SPEED_DIR_HIGH;
	pdu[i++] = LIGHT_BASE;
	pdu[i++] = LIGHT_MAX;
	pdu[i++] = LIGHT_MIN;
	//初始化电机参数
	i = motor1_state;
	int length = motor2_state-motor1_state;
	motorA_ptr = (Motor_struct*)&pdu[i];
	motorB_ptr = (Motor_struct*)&pdu[i+length];
	motorC_ptr = (Motor_struct*)&pdu[i+2*length];
	motorD_ptr = (Motor_struct*)&pdu[i+3*length];
	
	for( int j = 0; j < 4; j++){
		pdu[(i++)+j*length] = 0;   //节点状态
		pdu[(i++)+j*length] = FourWheer_Radiaus*10000; //车轮半径，乘以10000后
		pdu[(i++)+j*length] = REDUCTION_RATE*100;  //减速比，乘以100后
		pdu[(i++)+j*length] = j+1;   //CAN ID
		pdu[(i++)+j*length] = 1000;  //CAN波特率，除以100后
		pdu[(i++)+j*length] = 0;  //心跳
		pdu[(i++)+j*length] = 0;  //保留
		pdu[(i++)+j*length] = 0;  //目标转矩
		pdu[(i++)+j*length] = 0;
		pdu[(i++)+j*length] = 0;  //目标转速
		pdu[(i++)+j*length] = 0;
		pdu[(i++)+j*length] = 0; //目标位置
		pdu[i+j*length] = 0;
	}
	//初始化小车速度限幅参数
	i = car_max_lin_speed;
	union {
		float v;
		int16_t ud[2];
	}tmp;
	tmp.v = 0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = 0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i] = 0;
	

	
}
