#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "usartx.h"
#include "user_can.h"
#include "motor_data.h"


int g_nMotorLF_config = -1;
int g_nMotorLB_config = -1;  
int g_nMotorRF_config = -1;
int g_nMotorRB_config = -1;

union URcv_Vel_Data uVelLF;
union URcv_Vel_Data uVelLB;
union URcv_Vel_Data uVelRF;
union URcv_Vel_Data uVelRB;  //存放左右实时速度的变量

union URcv_ERROR_Data uError; // 接收驱动器的故障状态


int g_nReInitMotor_LB = 0;   //处理驱动器断电而开发板未断电，重新初始化驱动器的问题
int g_nReInitMotor_LF = 0;   //为1代表需要重新初始化，为0则代表不需要重新初始化
int g_nReInitMotor_RF = 0;
int g_nReInitMotor_RB = 0;


int g_nHeart_count_LB = 0;    //设置了心跳包，在规定次数内没产生心跳，则认为从机下线
int g_nHeart_count_LF = 0;    //将其状态设置为STATE_STOP
int g_nHeart_count_RF = 0;
int g_nHeart_count_RB = 0;

int g_nHeart_Lastcount_LB = 0;  //记录上次进入心跳包的计数值
int g_nHeart_Lastcount_LF = 0;
int g_nHeart_Lastcount_RF = 0;
int g_nHeart_Lastcount_RB = 0;




enum ENUM_HEART_STAT eLF_Motor_Heart = STATE_UNKNOW;
enum ENUM_HEART_STAT eLB_Motor_Heart = STATE_UNKNOW;
enum ENUM_HEART_STAT eRF_Motor_Heart = STATE_UNKNOW; 
enum ENUM_HEART_STAT eRB_Motor_Heart = STATE_UNKNOW;

enum ENUM_ERROR_STATE eLF_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eLB_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eRF_Motor_Error = ERROR_NONE;
enum ENUM_ERROR_STATE eRB_Motor_Error = ERROR_NONE;   // 驱动器故障码读取

/**********************************************************
* 函数功能： stm32底层的can通信协议初始化。
* 参数：     无
* 说明：     无
**********************************************************/
void Can_Driver_Init(void)
{

	GPIO_InitType GPIO_InitStructure;
	CAN_InitType CAN_InitStructure;
	CAN_FilterInitType CAN_FilterInitStructure;

#if CAN_RX0_INT_ENABLE
	NVIC_InitType NVIC_InitStructure;
#endif

	//对应的GPIO口和时钟初始化
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1, ENABLE); //使能CAN时钟
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE); //使能AFIO时钟

	//需要将CAN功能复用，映射到PD0和PD1上
	GPIO_ConfigPinRemap(GPIO_RMP2_CAN1, ENABLE);

	//初始化用到的GPIO口
	GPIO_InitStructure.Pin = GPIO_PIN_0;	  // PD0作为Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入模式
	GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_1;		// PD1作为Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出,使用复用CAN功能
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
	GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);

	//工作模式、波特率初始化
	// CAN单元设置
	CAN_InitStructure.TTCM = DISABLE;		  //非时间出发通信模式
	CAN_InitStructure.ABOM = DISABLE;		  //软件自动离线管理
	CAN_InitStructure.AWKUM = DISABLE;		  //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.NART = ENABLE;		  //禁止报文自动重传
	CAN_InitStructure.RFLM = DISABLE;		  //报文不锁定,新的覆盖旧的
	CAN_InitStructure.TXFP = DISABLE;		  //优先级由报文标识符决定
	CAN_InitStructure.OperatingMode = CAN_Operating_NormalMode; //模式设置
	CAN_InitStructure.RSJW = CAN_RSJW_1tq;	  //
	CAN_InitStructure.TBS1 = CAN_TBS1_3tq;
	CAN_InitStructure.TBS2 = CAN_TBS2_2tq;
	CAN_InitStructure.BaudRatePrescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);

	//设置CAN筛选器
	//配置过滤器
	CAN_FilterInitStructure.Filter_Num = 0;					 //过滤器0
	CAN_FilterInitStructure.Filter_Mode = CAN_Filter_IdMaskMode;	 //工作在标识符屏蔽模式下
	CAN_FilterInitStructure.Filter_Scale = CAN_Filter_32bitScale; // 32位
	CAN_FilterInitStructure.Filter_HighId = 0x0000;				 ////32位ID
	CAN_FilterInitStructure.Filter_LowId = 0x0000;
	CAN_FilterInitStructure.FilterMask_HighId = 0x0000; // 32位MASK
	CAN_FilterInitStructure.FilterMask_LowId = 0x0000;
	CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_Filter_FIFO0; //过滤器0关联到FIFO0
	CAN_FilterInitStructure.Filter_Act = ENABLE;				 //使能过滤器0
	CAN1_InitFilter(&CAN_FilterInitStructure);							 //过滤器初始化

	//开启CAN中断
#if CAN_RX0_INT_ENABLE

	CAN_INTConfig(CAN1, CAN_INT_FMP0, ENABLE); // FIFO0消息挂号中断允许.	接收到数据就会产生中断
	// CAN_ClearFlag(CAN1,CAN_IT_FMP0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
}




/**********************************************************
* 函数功能： can中断服务函数	
* 参数：     无
* 说明：     在中断函数中获取到实时反馈的速度
**********************************************************/
#if CAN_RX0_INT_ENABLE	//使能RX0中断函数
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CanRxMessage RxMessage;
	
	CAN_ReceiveMessage(CAN1, 0, &RxMessage);
	
	// 电机PDO反馈
	if (RxMessage.StdId >= 0x181 && RxMessage.StdId <= 0X1FF)
	{//< 电机PDO反馈
		int id = RxMessage.StdId - 0x181;
		int i = 0;
		MOTOR_TPDO* md;
		if (id < 10)
		{
			for (i = 0;i < MAX_MOTOR_NUMBER;i++)
			{
				if (id == mrd[i].d.mapping)
				{
					id = i+1;// 找到映射后的ID
					break;
				}
			}
			i = 0;
			mrd[id].d.online = 1;
			md = &mtd[id];
			md->d.heartbeat = 0;
			md->d.status.sd = RxMessage.Data[i++];
			md->d.status.sd |= RxMessage.Data[i++] << 8;
			md->d.current_velocity = RxMessage.Data[i++];
			md->d.current_velocity |= RxMessage.Data[i++] << 8;
			md->d.current_velocity |= RxMessage.Data[i++] << 16;
			md->d.current_velocity |= RxMessage.Data[i++] << 24;
		}
	}
	else
	{//< SDO 处理

	}

	
	
	CAN_ClearFlag(CAN1,CAN_INT_FMP0);
	
}
#endif

