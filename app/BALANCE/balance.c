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
#include "motor.h"
#include "RJ_JT.h"
#include "CircularCar.h"
LightTime light_time;

float Move_X = 0, Move_Y = 0, Move_Z = 0;
int g_nVol_get_Flag = 0;          //
float g_fltProprity_Voltage = 1; //
float Voltage = 0.0f;

SDOMessage receivedMsg, send0Msg, send1Msg;
MOTOR_PARA motor_para[MAX_MOTOR_NUMBER];


float Z_Radian_Max = 0;
float Z_Radian = 0;
float Z_Degree = 0;

uint8_t f1_cnt = 0;
uint32_t ultrasonic1_filtering[4]; 
uint8_t f2_cnt = 0;
uint32_t ultrasonic2_filtering[4];

uint8_t SPI_Master_Rx_Buffer;
int SPI_heartbeat = 0;
uint8_t SPI_ReadWriteCycle = 0;
EXIO_INPUT exio_input;
EXIO_OUTPUT exio_output;

struct
{
    unsigned char SW : 2;//《 急停开关
    unsigned char Rising : 1;
    unsigned char Descending : 1;

    unsigned char estop_soft : 1;//< 软急停
    unsigned char estop_soft_old : 1;//< 软急停
} emergency_stop;
uint8_t Torque_sdo1[1][8] 		= {0x2B, 0x15, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}; 	//速度模式下力矩（DIFFCAR）
uint8_t Torque_sdo2[1][8] 		= {0x2B, 0x15, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00}; 	//速度模式下力矩（DIFFCAR）
uint8_t Torque_sdo[1][8] 		= {0x2B, 0x15, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00}; 	//速度模式下力矩
uint8_t Speed_sdo[1][8] 		= {0x2B, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};	//力矩模式下速度
uint8_t Torque_Mode_Sdo[1][8] 	= {0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};  	//设置力矩模式
uint8_t Speed_Mode_Sdo[1][8] 	= {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};  	//设置速度模式
static uint8_t battery_send_frame_num = 0;

/**************************************************************************
函数功能：AKM位置模式
入口参数：无
返回  值：无
**************************************************************************/
void ServoPulse_Enable()
{
    //g_nDirector为手柄CH1与基础值作差的通道值，取值范围为±784
    int CurrentPos = (int)(pdu[motor1_position2_feedback] << 16 | pdu[motor1_position1_feedback]);
    int	diff_pos = Servo_pulse - CurrentPos;
    static bool front_bit4 = 0;
    unsigned char flag = (mtd[0].d.status.sw >> 12) & 0x0F;

    switch(pdu[motor1_type])
    {
        case servo_zlac:
            mrd[0].d.ctrl.bit.bit4 = 0;

            if(diff_pos != 0)
            {
                //如果移动到位，取消急停，继续发送使能信号（发送一次）
                if((flag == 0xA) || (flag == 0x6))
                {
                    mrd[0].d.ctrl.bit.bit4 = 1;
                    mrd[0].d.ctrl.bit.bit8 = 0;

                    if(front_bit4 == 1)
                    {
                        mrd[0].d.ctrl.bit.bit4 = 0;
                    }

                    front_bit4 = mrd[0].d.ctrl.bit.bit4;
                }
                else if(myabs(diff_pos) < 300) //如果出现了临界情况
                {
                    mrd[0].d.ctrl.bit.bit4 = 0;//急停
                    mrd[0].d.ctrl.bit.bit8 = 1;
                }
            }

            break;

        case servo_wanze:
            if(myabs(diff_pos) > 1)
            {
                uint8_t Init_sdo[][8] =
                {
                    {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00},
                    {0x2B, 0x40, 0x60, 0x00, 0x1F, 0x00, 0x00, 0x00},
                };
                Add_Sdo_Linked_List(pdu[motor1_CAN_id], Init_sdo, sizeof(Init_sdo) / sizeof(Init_sdo[0]));
            }

            break;

        default:
            break;
    }
}

/*---------------------------一些功能函数--------------------------------*/
/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{
    return (a < 0) ?  -a : a;
}



/**************************************************************************
函数功能：浮点型数据取绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
    return (insert < 0) ?  -insert : insert;
}


/**************************************************************************
函数功能：限幅函数，设定高低阈值
入口参数：幅值
返回  值：
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

void setGPIO(GPIO_Module* GPIOx, uint16_t GPIO_Pin, uint8_t value)
{
    if (value)
    {
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    }
    else
    {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
}

/**************************************************************************
函数功能：电池信息处理函数
入口参数：void
返回  值：void
**************************************************************************/
void BatteryInformation(void)
{
    static int bt = 0;
    static int bt_times = 100;
    short MOVE_XorZ = 0;
    short temp;

    if (uart4_recv_flag) // 检查 UART4 接收标志，如果为真，则继续处理接收到的数据
    {
        uart4_recv_flag = 0;

        switch (pdu[BatteryManufacturer]) // 根据电池制造商设置行为
        {
            case 1://< 深圳市锂神科技有限公司 24Voltage
                if (myabs((short)pdu[target_linear_speed]) > myabs((short)pdu[target_yaw_speed])) // 比较小车的线速度和角速度，选择最大的一个
                {
                    MOVE_XorZ = (short)pdu[target_linear_speed];
                    temp = (short)pdu[max_linear_speed];
                }
                else
                {
                    MOVE_XorZ = (short)pdu[target_yaw_speed];
                    temp = (short)pdu[max_yaw_speed];
                }

                if (MOVE_XorZ < 0)  // 绝对值化移动速度
                {
                    MOVE_XorZ = - MOVE_XorZ;
                }// 根据移动速度调整刷新间隔

                bt_times = 	MOVE_XorZ > (short)(temp / 1.2) ? 8 :
                            MOVE_XorZ > (short)(temp / 2) 	? 16 :
                            MOVE_XorZ > (short)(temp / 4) 	? 32 :
                            MOVE_XorZ > (short)(temp / 8) 	? 64 : 100;

                if (uart4_recv_data[0] == 0x3B) // 检查接收到的 UART4 数据头是否正确
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

            default: //48Voltage
                bt_times = 100;//< 1000ms刷新1次

                if (uart4_recv_data[0] == 0x7e)
                {
                    uint32_t tmp = 0;
                    int i = 0;
                    pdu[BatteryStatus] = 1;//< 电池读取成功
                    pdu[BatteryQuantity] = (uint16_t)uart4_recv_data[107] | (((uint16_t)uart4_recv_data[108]) << 8);//< 电池电量
                    pdu[BatteryQuantity] *= 10;
                    i = 97;
                    tmp = (uint32_t)uart4_recv_data[i++];
                    tmp |= (uint32_t)uart4_recv_data[i++] << 8;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 16;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 24;
                    pdu[BatteryVoltage] = (int16_t)((int)tmp * 0.1); //< 电池电压
                    tmp = (uint32_t)uart4_recv_data[i++];
                    tmp |= (uint32_t)uart4_recv_data[i++] << 8;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 16;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 24;
                    pdu[BatteryCurrent] = (int16_t)((int)tmp * 0.1);//< 电池电流
                    pdu[BatteryTemperature] = (uint16_t)uart4_recv_data[77] | (((uint16_t)uart4_recv_data[78]) << 8);//< 电池温度
                }

                break;
        }
    }

    bt++;

    if (bt >= bt_times)
    {
        bt = 0;
        GPIO_SetBits(UARTFour_485en_GPIO, UARTFour_485enPin);	// 打开 UART4 485 使能引脚
        DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);    // 关闭 DMA2 通道5, UART4_TX
        DMA_SetCurrDataCounter(UARTFour_Tx_DMA_Channel, battery_send_frame_num);  // 传输数量寄存器只能在通道不工作(DMA_CCRx的EN=0)时写入
        DMA_EnableChannel(UARTFour_Tx_DMA_Channel, ENABLE);    // 开启 DMA2 通道5, UART4_TX
    }
}
#if(CARMODE != Diff)
/**************************************************************************
函数功能：电源控制
入口参数：void
返回  值：void
**************************************************************************/
void PowerControl(void)
{
    static union
    {
        struct
        {
            uint16_t jdq1 : 1;//< 左电机电源 JDQ1_EN
            uint16_t jdq2 : 1;//< 右电机电源 JDQ2_EN
            uint16_t p12v : 1;//< 12V电源 YL_7
            uint16_t p19v : 1;//< 19V电源 YL_6
        } bit;
        uint16_t pc;
    } power_ctrl;

    if (pdu[power_control] != power_ctrl.pc)
    {
        power_ctrl.pc = pdu[power_control];
        setGPIO(JDQ_PORT, JDQ1_PIN, power_ctrl.bit.jdq1);
        setGPIO(JDQ_PORT, JDQ2_PIN, power_ctrl.bit.jdq2);
        setGPIO(YL_7_GPIO, YL_7_Pin, power_ctrl.bit.p12v);
        setGPIO(YL_6_GPIO, YL_6_Pin, power_ctrl.bit.p19v);
    }
}


/**************************************************************************
函数功能：SPI读写函数
入口参数：void
返回  值：void
**************************************************************************/
void SPI1_ReadWriteByte(void)
{
    if (!SPI_ReadWriteCycle)
    {
        uint16_t retry = 0;
        GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_SET);
        pdu[exio_input_status] = exio_input.input = SPI_Master_Rx_Buffer;

        for (retry = 0; retry < 1000;) // 空循环，用于等待
        {
            retry++;
        }

        retry = 0;

        while (!(SPI_MASTER->STS & 1 << 1)) //等待发送区空
        {
            retry++;

            if (retry > 2000) // 如果等待时间超过 2000，则表示出现错误
            {
                pdu[car_error_messages] = spi_error;//< SPI错误
                return;// 返回，退出函数
            }
        }

        if (((pdu[control_mode] == control_mode_ros) || (pdu[control_mode] == control_mode_ipc))
                && (robot_control.bit.light_ctrl_en)) //< 默认灯控制权
        {
            pdu[light_control] = Receive_Data[2];
        }
        else
        {
            pdu[light_control] = exio_output.output;
        }

        SPI_MASTER->DAT = pdu[light_control];
        SPI_ReadWriteCycle = 1; // 标记 SPI 读写周期开始
        SPI_heartbeat = 0;
    }

    if (SPI_ReadWriteCycle)  // 如果 SPI 读写周期已经开始
    {
        SPI_heartbeat ++;

        if (SPI_heartbeat > 20) // 重置心跳计数器和读写周期状态
        {
            SPI_heartbeat = 0;
            SPI_ReadWriteCycle = 0;
        }
    }
}
#endif


/**************************************************************************
函数功能：电池初始化相关
入口参数：
返回  值：
**************************************************************************/
void BatteryInfoInit(void)
{
    battery_send_frame_num = 0;

    switch (pdu[BatteryManufacturer])
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
函数功能：核心控制相关
入口参数：
返回  值：
**************************************************************************/
uint8_t tmp1 = 0;
void Balance_task(void* pvParameters)
{


    while (1)
    {
        rt_thread_delay(50);   //< 5ms
        //存疑，中菱说10ms下发速度比较好
        /*RunningTest*/
        tmp1++;
        if (tmp1 == 100)
        {
            //运行指示灯
            LedBlink(LED17_GPIO, LED17_PIN);//  灯闪烁 用于判断此函数是否正常运行
            tmp1 = 0;
        }        
        SetReal_Velocity(); //获取到设定的速度和方向值
				
		// 阿克曼 有三个电机 ，只有前轮一个电机转向，位置模式，  其它电机 全部速度模式   
        Kinematic_Analysis(pdu[target_linear_speed], Move_Y, pdu[target_yaw_speed]);//将线速度和角速度转化为电机的输入值
       
		// 用于解决电机 编号的问题，   通过 判断2号电机的模式(),  最大的原因   圆盘的驱动器 是一拖二，其它都 是一拖一
		uint16_t x_sport_mode = (pdu[car_type] == Diff_Car) ? pdu[motor1_sport_mode] : pdu[motor2_sport_mode];
				
		//目前最有效的结果，就是解决了 中凌电机的 电流限制问题
        ClassificationOfMotorMotionModes(x_sport_mode);	//电机运动模式分类
    }
}

/**************************************************
* 函数功能：获取到设定的速度和方向值
**************************************************/
short test_Raw = 0;
short test_Linear = 0;
void SetReal_Velocity()
{
    if (Soft_JT_Flag) //< 急停
    {
        pdu[target_linear_speed] = Move_Y = pdu[target_yaw_speed] = pdu[target_angle] = Servo_pulse = 0;
        //  target_linear_speed      目标线速度  X轴方向
        //  Move_Y                   目标线速度  Y轴方向
        //  pdu[target_yaw_speed]    目标线速度  Z轴方向   车身旋转
        //  pdu[target_angle]        车的转角，只对 阿克曼 有效
        //  Servo_pulse     阿克曼前轮电机脉冲数  只对阿克曼有效， 用于前轮转向
    }
    else
    {
        switch(pdu[control_mode])  //0:未知，1:航模，2:ROS，3:上位机，4:其他
        {
            case control_mode_ros:
                if(ROS_RecvFlag)
                {
                    test_Linear = pdu[target_linear_speed] =0- ConvertBytesToShort(Receive_Data[3], Receive_Data[4]);	//X轴速度(10-3m/s)
									
                    Move_Y = ConvertBytesToShort(Receive_Data[5], Receive_Data[6]);	//Y轴速度(小车在Y轴速度为0)
									
                    test_Raw = pdu[target_yaw_speed] =(0- ConvertBytesToShort(Receive_Data[7], Receive_Data[8]));	//Z轴速度(10-3rad/s)
									
									
									/****************** 阿克曼 专用 5个  ******************/
					if(pdu[car_type] == Akm_Car){
									  // 阿克曼 专用 
                        Z_Radian = atan(pdu[car_wheelbase] * MAGNIFIC_10000x_DOWN * pdu[target_yaw_speed] / (pdu[target_linear_speed] + EPSILON));//阿克曼前轮转角弧度
									      // 阿克曼 专用 
                        Z_Radian = fmin(Z_Radian_Max, fmax(-Z_Radian_Max, Z_Radian));//转角弧度限幅(单位：0.1rad) 阿克曼
									       // 阿克曼 专用 
                        pdu[target_angle] = (short)(Z_Radian * RADtoANG * MAGNIFIC_10x_UP); //更正转角  阿克曼				
									      // 阿克曼 专用 
                        pdu[target_yaw_speed] = (short)(tan(Z_Radian) * (pdu[target_linear_speed] + EPSILON) / (pdu[car_wheelbase] * MAGNIFIC_10000x_DOWN));	//修正角速度
									      // 阿克曼 专用 
                        Servo_pulse = (pdu[target_angle] / DEGREE_MAX) * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];
					}
									
									
                    ROS_RecvFlag = false;
                }

                break;

            case control_mode_ipc:   //qth 上位机
				//未完善
                //X 轴
                pdu[target_linear_speed] = VirtuallyLinearVelocityGet();						//线速度单位10(-3)m/s
                 //z 轴
				pdu[target_yaw_speed] = VirtuallyAngularVelocityGet(pdu[target_linear_speed]);	//角速度单位10(-3)rad/s
                break;

            case control_mode_remote:  //航模遥控器
				//X 轴
                test_Linear = pdu[target_linear_speed] = LinearVelocityGet();							//线速度单位10(-3)m/s
				//z 轴
                test_Raw = pdu[target_yaw_speed] = AngularVelocityGet(pdu[target_linear_speed]);	//角速度单位10(-3)rad/s
                break;

            case control_mode_other:  //其它控制
            case control_mode_unknown: //未知  清零
                pdu[target_linear_speed] = Move_Y = pdu[target_yaw_speed] = pdu[target_angle] = Servo_pulse = 0;
                break;

            default:
                break;
        }

        //小车线速度/角速度/前轮转角限幅
        if(pdu[car_type] != RC_Car)
        {   
			//限幅 防止超限，最大最小值  由用户 设置
            pdu[target_linear_speed] = limit_value((short)pdu[target_linear_speed], (short)pdu[max_linear_speed]);
            pdu[target_yaw_speed] = limit_value((short)pdu[target_yaw_speed], (short)pdu[max_yaw_speed]);
            pdu[target_angle] = limit_value((short)pdu[target_angle], (short)pdu[max_angle]);
        }
    }
}

/**************************************************************************
函数功能：将线速度和角速度转化为电机的输入值
入口参数：角速度线速度
返回  值：无
**************************************************************************/
short testm1;
short testm2;

void Kinematic_Analysis(short Vx, float Vy, short Vz)
{
    float common_part = LineSpeedToMotorSpeed * MAGNIFIC_1000x_DOWN;	// 提取共同部分
    short wheel_distance_factor;										// 提取车轮与轴间的距离部分
    float correction_factor;										   // 校正参数
    short m;
    uint16_t line_, angle_;

    switch (pdu[car_type]) //运动学分析
    {
        case Akm_Car:   //阿克曼
            wheel_distance_factor = (short)(Vz * pdu[car_tread] * MAGNIFIC_10000x_DOWN / 2.0f);
            pdu[motor2_target_rpm] = (short)((Vx - wheel_distance_factor) * common_part);	// 单位：r/min
            pdu[motor3_target_rpm] = -(short)((Vx + wheel_distance_factor) * common_part);	// 计算 motor2 和 motor3 的目标转速
            ServoPulse_Enable();
            break;

        case FourWheel_Car://四轮室外差速
            wheel_distance_factor = (Vz * (pdu[car_tread] + pdu[car_wheelbase]) * MAGNIFIC_10000x_DOWN / 2.0f);
            correction_factor = 0.578f;
            pdu[motor2_target_rpm] = pdu[motor1_target_rpm] = (short)((Vx - correction_factor * wheel_distance_factor) * common_part);
            pdu[motor4_target_rpm] = pdu[motor3_target_rpm] = -(short)((Vx + correction_factor * wheel_distance_factor) * common_part);
            break;

        case Diff_Car:   //圆形差速度底盘
            pdu[car_wheelbase] = 0;//确保圆形底盘轴距为0，两轮差速运动学模型一致
        case TwoWheel_Car: //两轮室内差速度
        case Tank_Car:      //坦克
            wheel_distance_factor = (Vz * (pdu[car_tread] + pdu[car_wheelbase]) * MAGNIFIC_10000x_DOWN / 2.0f);
            pdu[motor1_target_rpm] = (short)((Vx - wheel_distance_factor) * common_part);
            pdu[motor2_target_rpm] = -(short)((Vx + wheel_distance_factor) * common_part);
            testm1 = pdu[motor1_target_rpm];
            testm2 = pdu[motor2_target_rpm];				
            break;

        case RC_Car:
            m = (100 + (short)pdu[rc_magnification]) * 10;
            line_ = (short)(Vx * MAGNIFIC_1000x_DOWN * MYHALF * m) + 3000;//线速度
            angle_ = (short)(Vz * MAGNIFIC_1000x_DOWN * m) + 3000;//角度
            pdu[target_linear_speed] = line_;
            pdu[target_yaw_speed] = angle_;
            RCCAR_Process(line_, angle_);//发送PWM给电机
            break;

        case Charger://充电桩

        default:
            break;
    }
}

/**************************************************************************
函数功能：电机运动模式分类,目前是根据遥控器档位给力矩
入口参数：sport_mode
返 回 值：无
**************************************************************************/
uint16_t test_Torque_SWB = 0;
void ClassificationOfMotorMotionModes(uint16_t sport_mode)
{
    static int mode_count = 0;
    bool torque_switch = false; //速度模式下力矩（电流）切换标志位
    static uint16_t Torque_SWB = 0, Torque_Max = 0, Last_Gear_value = 0, Last_Torque_value = 0;
    uint8_t Speed_Max[4] =  //力矩模式下最大速度挡位，200A地址对应的最大转速为U16，需要进行类型转换
    {
        myabs(pdu[motor2_target_rpm]) & 0xFF, (myabs(pdu[motor2_target_rpm]) >> 8) & 0xFF,
        myabs(pdu[motor3_target_rpm]) & 0xFF, (myabs(pdu[motor3_target_rpm]) >> 8) & 0xFF,
    };

    switch (sport_mode) //模式判断
    {
        case speed_mode:
            if(pdu[GearPosition_value] - Last_Gear_value) //对应遥控器上的高中低三个档位发生改动
            {
                if (Abs_int(pdu[GearPosition_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_LOW_GEAR;//速度模式下最大力矩挡位
                }
                else if (Abs_int(pdu[GearPosition_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_MIDDLE_GEAR;
                }
                else if (Abs_int(pdu[GearPosition_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_HIGH_GEAR;  //通过SDO  发过电机力矩  对象字典   只有中凌有，万泽没有此选项。
                }
								test_Torque_SWB = Torque_SWB;
                torque_switch = true;
            }

#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)			
			//FS-i6s左边的旋钮，电流被分成九档
            if ((Last_Torque_value == pdu[rc_max_value]) && ((pdu[Torque_value] - Last_Torque_value) < 0) && (pdu[torque_cofficient] < TORQUE_COEFFICIENT_MAX))
            {
                pdu[torque_cofficient] ++;
                torque_switch = true;
            }
            else if ((Last_Torque_value == pdu[rc_min_value]) && ((pdu[Torque_value] - Last_Torque_value) > 0) && (pdu[torque_cofficient] > TORQUE_COEFFICIENT_MIN))
            {
                pdu[torque_cofficient] --;
                torque_switch = true;
            }
#elif(REMOTE_TYPE == REMOTE_TYPE_HT8A)
            if (pdu[Torque_value] - Last_Torque_value) //对应遥控器上的高中低三个力矩档位发生改动
            {
                if (Abs_int(pdu[Torque_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_MIN;//速度模式下最小力矩挡位
                }
                else if (Abs_int(pdu[Torque_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_BASE;
                }
                else if (Abs_int(pdu[Torque_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_MAX; //通过SDO  发过电机力矩  对象字典   只有中凌有，万泽没有此选项。
                }
                torque_switch = true;
            }
#endif						
			//更新值，作为对比
            Last_Gear_value = pdu[GearPosition_value];
            Last_Torque_value = pdu[Torque_value];

						
			//sdo  把电流 值 发送出去
            if(torque_switch)
            {
                Odom_distance_mm = 0; //里程计清零。存疑
                Torque_Max = pdu[torque_cofficient] * Torque_SWB;	//2015h（电流）范围为0-300
                Torque_sdo[0][4] = Torque_Max & 0xFF;
                Torque_sdo[0][5] = ((Torque_Max) >> 8) & 0xFF;

                switch (pdu[car_type])
                {
                    case Akm_Car:
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        break;

                    case Diff_Car:
                        Torque_sdo2[0][4] = Torque_sdo1[0][4] = Torque_Max & 0xFF;
                        Torque_sdo2[0][5] = Torque_sdo1[0][5] = ((Torque_Max) >> 8) & 0xFF;
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_sdo1, sizeof(Torque_sdo1) / sizeof(Torque_sdo1[0]));
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_sdo2, sizeof(Torque_sdo2) / sizeof(Torque_sdo2[0]));
                        break;

                    case TwoWheel_Car:
                    case Tank_Car:
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        break;

                    case FourWheel_Car:
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor4_CAN_id], Torque_sdo, sizeof(Torque_sdo) / sizeof(Torque_sdo[0]));
                        break;

                    default:
                        break;
                }
            }

            break;
             //  目前只是简单测试过，没有速度模式好用，所以此方案暂时废弃
        case torque_mode:
            switch (pdu[car_type])
            {
                case Akm_Car:
                    if((mode_count == 1) && (pdu[Forward_value] != pdu[rc_base_value])) //非停止状态下切换成转矩模式，不带刹车
                    {
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        mode_count = 0;
                    }

                    if((pdu[Forward_value] == pdu[rc_base_value]) && (myabs(pdu[motor2_rpm_feedback]) < 2)) //停止下来切换成速度模式，带刹车
                    {
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        mode_count = 1;
                    }

                    Speed_sdo[0][4] = Speed_Max[0];
                    Speed_sdo[0][5] = Speed_Max[1];
                    Add_Sdo_Linked_List(pdu[motor2_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    Speed_sdo[0][4] = Speed_Max[2];
                    Speed_sdo[0][5] = Speed_Max[3];
                    Add_Sdo_Linked_List(pdu[motor3_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    pdu[motor2_target_torque] = pdu[motor2_target_rpm] * MAGNIFIC_10x_UP; //乘以10是因为ZLAC中目标转矩与目标速度取值范围差了10倍
                    pdu[motor3_target_torque] = pdu[motor3_target_rpm] * MAGNIFIC_10x_UP;
                    break;

                case FourWheel_Car:
                    if((mode_count == 1) && (pdu[Forward_value] != pdu[rc_base_value])) //非停止状态下切换成转矩模式，不带刹车
                    {
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor4_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        mode_count = 0;
                    }

                    if((pdu[Forward_value] == pdu[rc_base_value]) && (myabs(pdu[motor2_rpm_feedback]) < 2)) //停止下来切换成速度模式，带刹车
                    {
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor4_CAN_id], Speed_Mode_Sdo, sizeof(Speed_Mode_Sdo) / sizeof(Speed_Mode_Sdo[0]));
                        mode_count = 1;
                    }

                    Speed_sdo[0][4] = Speed_Max[0];
                    Speed_sdo[0][5] = Speed_Max[1];
                    Add_Sdo_Linked_List(pdu[motor1_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    Add_Sdo_Linked_List(pdu[motor2_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    Speed_sdo[0][4] = Speed_Max[2];
                    Speed_sdo[0][5] = Speed_Max[3];
                    Add_Sdo_Linked_List(pdu[motor3_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    Add_Sdo_Linked_List(pdu[motor4_CAN_id], Speed_sdo, sizeof(Speed_sdo) / sizeof(Speed_sdo[0]));
                    pdu[motor1_target_torque] = pdu[motor1_target_rpm] * MAGNIFIC_10x_UP; //乘以10是因为ZLAC中目标转矩与目标速度取值范围差了10倍
                    pdu[motor2_target_torque] = pdu[motor2_target_rpm] * MAGNIFIC_10x_UP;
                    pdu[motor3_target_torque] = pdu[motor3_target_rpm] * MAGNIFIC_10x_UP;
                    pdu[motor4_target_torque] = pdu[motor4_target_rpm] * MAGNIFIC_10x_UP;
                    break;

                case TwoWheel_Car://暂无力矩模式
                case Diff_Car:
                case Tank_Car:
                    break;

                default:
                    break;
            }

        default:
            break;
    }
}


/**************************************************
* 函数功能：得到线速度
* 返 回 值：返回线速度
**************************************************/
//  输入对应遥控器的通道值
short LinearVelocityGet(void)
{
    uint16_t VelocityCoefficient = (myabs(pdu[GearPosition_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR) ? pdu[linear_low] : //线速度系数
                (myabs(pdu[GearPosition_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR) ? pdu[linear_middle] : pdu[linear_high];
    float VelocityTemp = pdu[robot_forward_direction] * (pdu[Forward_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference];

    switch(pdu[car_type])
    {
        case Akm_Car:		// 阿克曼
        case FourWheel_Car:	// 室外差速四驱车
        case Diff_Car:		// 差速车(圆形底盘)
        case TwoWheel_Car: 	// 室内差速二驱车
        case Tank_Car:   	// 坦克车
        case RC_Car:   		// RC车
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?//使能拨杆
                   (short)(VelocityCoefficient * VelocityTemp) : 0;

        default:
            return 0;
    }
}

/**************************************************
* 函数功能：得到角速度
* 返 回 值：返回角速度
**************************************************/

//  输入对应遥控器的通道值
short AngularVelocityGet(short linearValue)
{
    uint16_t YawCoefficient = (pdu[GearPosition_value] == pdu[rc_min_value]) ? pdu[angular_low] : //角速度系数
                              (pdu[GearPosition_value] == pdu[rc_base_value]) ? pdu[angular_middle] : pdu[angular_high];
    float YawTemp = -(pdu[robot_forward_direction] * (pdu[Turn_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference]); //角速度与转向通道值正方向相反，归一法(-1,1)

    switch(pdu[car_type])
    {
        case Akm_Car:		// 阿克曼
            pdu[target_angle] = round(YawTemp * DEGREE_MAX); //前轮转角
            Servo_pulse = YawTemp * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];//脉冲
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(linearValue * YawTemp / AKM_RAW_FACTOR) : 0; // Z轴角速度

        case Diff_Car:		// 差速车(圆形底盘)
        case FourWheel_Car:	// 室外差速四驱车
        case TwoWheel_Car: 	// 室内差速二驱车
        case Tank_Car :   	// 坦克车
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(YawCoefficient * YawTemp / PI * 4) : 0; // Z轴角速度
            //系数存疑

        default:
            return 0;
    }
}

/**************************************************
* 函数功能：得到线速度
* 返 回 值：返回角速度
**************************************************/

// 输入为上位机的虚拟手柄通道值

short VirtuallyLinearVelocityGet(void)
{
    uint16_t VelocityCoefficient = (pdu[virtually_rc_ch6_value] == pdu[rc_min_value]) ? pdu[linear_low] : //线速度系数
                                   (pdu[virtually_rc_ch6_value] == pdu[rc_base_value]) ? pdu[linear_middle] : pdu[linear_high];
    float VelocityTemp = pdu[robot_forward_direction] * (pdu[virtually_rc_ch3_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference];

    switch(pdu[car_type])
    {
        case Akm_Car:		// 阿克曼
        case FourWheel_Car:	// 室外差速四驱车
        case Diff_Car:		// 差速车(圆形底盘)
        case TwoWheel_Car: 	// 室内差速二驱车
        case Tank_Car:   	// 坦克车
        case RC_Car:   		// RC车
            return (myabs(pdu[virtually_rc_ch7_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(VelocityCoefficient * VelocityTemp) : 0;

        default:
            return 0;
    }
}

/**************************************************
* 函数功能：得到角速度
* 返 回 值：返回角速度
**************************************************/

// 输入为上位机的虚拟手柄通道值
short VirtuallyAngularVelocityGet(short linearValue)
{
    uint16_t YawCoefficient = (pdu[virtually_rc_ch6_value] == pdu[rc_min_value]) ? pdu[angular_low] : //角速度系数
                              (pdu[virtually_rc_ch6_value] == pdu[rc_base_value]) ? pdu[angular_middle] : pdu[angular_high];
    float YawTemp = -(pdu[robot_forward_direction] * (pdu[virtually_rc_ch1_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference]); //角速度与转向通道值正方向相反

    switch(pdu[car_type])
    {
        case Akm_Car:		// 阿克曼
            pdu[target_angle] = round(YawTemp * DEGREE_MAX); //前轮转角
            Servo_pulse = YawTemp * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];//脉冲
            return (short)(linearValue * YawTemp / AKM_RAW_FACTOR); // Z轴角速度
            break;
        case Diff_Car:		// 差速车(圆形底盘)
        case FourWheel_Car:	// 室外差速四驱车
        case TwoWheel_Car: 	// 室内差速二驱车
        case Tank_Car:   	// 坦克车
            return (myabs(pdu[virtually_rc_ch7_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                (short)(YawCoefficient * YawTemp / PI * 4) : 0; // Z轴角速度
            break;
        default:
            return 0;
    }
}



/**************************************************************************
函数功能：充电桩初始化
入口参数：
返回  值：
**************************************************************************/
void ChargerBalanceInit(void)
{
    if (pdu[car_type] == Charger)
    {
        IrDA_TX = 0;
        FAN1 = 1;
        FAN2 = 1;
        Relay_Init();
        //Key_Init();
        RGB_Init();
        LimitSwitch_Init();
        ChargeDetection_Init();
    }
}

