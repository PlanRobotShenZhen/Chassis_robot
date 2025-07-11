#include "remote.h"
#include "usartx.h"
#include "rtthread.h"
#include "485_address.h"
#include "motor_data.h"
#include "RJ_JT.h"
#include "balance.h"
#include "robot_select_init.h"

bool Sbus_Data_Parsing_Flag = false;	//航模数据需要处理标志位

/**************************************************************************
函数功能：ROS--Usart3 、手柄控制--Usart5相关
入口参数：
返回  值：
**************************************************************************/
void Remote_Task(void *pvParameters)
{
    while (1)
    {
        rt_thread_delay(TIM);
        Usart3_Recv();			//串口3(ROS)接收数据
        Usart3_Send();    		//串口3(ROS)发送数据
        Sbus_Data_Parsing();	//解析航模数据
        MotorEnableFunction();	//电机使能   0 -6 -7 -15  控制字的切换
    }
}


/**************************************************
* 函数功能：将sbus信号转换为通道值
* 参    数：Uart5_Buffer为接收到的串口数据
* 返 回 值：无
**************************************************/
int REMOTE_Count = 0;
void Sbus_Data_Parsing(void)
{
//这里用的是SBUS协议，参考网址：https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
    /*SBUS共25个字节，14ms发送一次，发送帧包括1个起始位+8位数据位+1个偶校验+2个停止位。
	用DMA到Uart5_Buffer只保存25个8位数据位
    第2个字节~第23个字节为对应16个通道，每个通道11bit。22*8=16*11，每个通道取值范围为0~2^11-1=0-2047，本款取值240-1807。
    高位先发，ch1的11位=data2的低3位+data1的8位，依此类推。第24个字节为状态字flag，0代表所有数据正确，1代表失败。
    */
	// 如果当前使用的是FS-i6型号的遥控器
#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)
	 // 遥控器计数器：用于检测遥控器是否失去连接
    // 当计数器小于100秒（假设DELAY_COUNT_1S是1秒的计数单位）时增加计数器
    if(REMOTE_Count < DELAY_COUNT_1S * 100)
    {
        REMOTE_Count++;
    }
#endif
	// 当SBUS数据解析标志被置位（表示有新的遥控器数据需要处理）
    if (Sbus_Data_Parsing_Flag)
    {
		// 清除数据解析标志，等待下一次数据
        Sbus_Data_Parsing_Flag = false;

        //0起始字节为0F，23标志正常为00，24结束字节00
        //FS-i6S遥控器最多只支持到前10个通道
        // //HT-8A遥控器最多支持8通道
        //Uart5_Buffer[23]理论正常应该为00，但HT-8A显示03
        if (Uart5_Buffer[0] == 0x0F && Uart5_Buffer[24] == 0x00)
        {
			 // 解析16个通道的数据（每个通道11位数据，0x07FF是11位的掩码）
			// 通道1：组合字节1和字节2的数据
            pdu[rc_ch1_value] = ((int16_t)Uart5_Buffer[1] >> 0 | ((int16_t)Uart5_Buffer[2] << 8)) & 0x07FF;
            pdu[rc_ch2_value] = ((int16_t)Uart5_Buffer[2] >> 3 | ((int16_t)Uart5_Buffer[3] << 5)) & 0x07FF;
            pdu[rc_ch3_value] = ((int16_t)Uart5_Buffer[3] >> 6 | ((int16_t)Uart5_Buffer[4] << 2) | (int16_t)Uart5_Buffer[5] << 10) & 0x07FF;
            pdu[rc_ch4_value] = ((int16_t)Uart5_Buffer[5] >> 1 | ((int16_t)Uart5_Buffer[6] << 7)) & 0x07FF;
            pdu[rc_ch5_value] = ((int16_t)Uart5_Buffer[6] >> 4 | ((int16_t)Uart5_Buffer[7] << 4)) & 0x07FF;
            pdu[rc_ch6_value] = ((int16_t)Uart5_Buffer[7] >> 7 | ((int16_t)Uart5_Buffer[8] << 1) | (int16_t)Uart5_Buffer[9] << 9) & 0x07FF;
            pdu[rc_ch7_value] = ((int16_t)Uart5_Buffer[9] >> 2 | ((int16_t)Uart5_Buffer[10] << 6)) & 0x07FF;
            pdu[rc_ch8_value] = ((int16_t)Uart5_Buffer[10] >> 5 | ((int16_t)Uart5_Buffer[11] << 3)) & 0x07FF;
            pdu[rc_ch9_value] = ((int16_t)Uart5_Buffer[12] << 0 | ((int16_t)Uart5_Buffer[13] << 8)) & 0x07FF;
            pdu[rc_ch10_value] = ((int16_t)Uart5_Buffer[13] >> 3 | ((int16_t)Uart5_Buffer[14] << 5)) & 0x07FF;
            pdu[rc_ch11_value] = ((int16_t)Uart5_Buffer[14] >> 6 | ((int16_t)Uart5_Buffer[15] << 2) | (int16_t)Uart5_Buffer[16] << 10) & 0x07FF;
            pdu[rc_ch12_value] = ((int16_t)Uart5_Buffer[16] >> 1 | ((int16_t)Uart5_Buffer[17] << 7)) & 0x07FF;
            pdu[rc_ch13_value] = ((int16_t)Uart5_Buffer[17] >> 4 | ((int16_t)Uart5_Buffer[18] << 4)) & 0x07FF;
            pdu[rc_ch14_value] = ((int16_t)Uart5_Buffer[18] >> 7 | ((int16_t)Uart5_Buffer[19] << 1) | (int16_t)Uart5_Buffer[20] << 9) & 0x07FF;
            pdu[rc_ch15_value] = ((int16_t)Uart5_Buffer[20] >> 2 | ((int16_t)Uart5_Buffer[21] << 6)) & 0x07FF;
            pdu[rc_ch16_value] = ((int16_t)Uart5_Buffer[21] >> 5 | ((int16_t)Uart5_Buffer[22] << 3)) & 0x07FF;
					
#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)
            REMOTE_Count = 0;
            pdu[control_mode] = control_mode_remote;
            pdu[rc_connect_state] = rc_state_success;
        }
    }
    else if (REMOTE_Count > 200)//解码超时
    {
        pdu[rc_connect_state] = rc_state_failed;
        if (pdu[control_mode] != control_mode_ros)
        {
            pdu[control_mode] = control_mode_unknown;
        }
    }

#elif(REMOTE_TYPE == REMOTE_TYPE_HT8A)
        }
        //该遥控器的接收端未接收到信号时一直发待机状态，故只有remote和ros两种控制模式
        //遥控器默认状态
		// 检查遥控器是否处于默认状态（所有通道在中间位置，使能通道为最小值，急停通道为最小值）
       // pdu[Forward_value] == pdu[rc_base_value] && pdu[Turn_value] == pdu[rc_base_value] &&
       if ( pdu[Enable_value] == pdu[rc_min_value] && pdu[EmergencyStop_value] == pdu[rc_min_value])
        {
            //三秒默认状态视为失去遥控器控制
            if (REMOTE_Count > (DELAY_COUNT_1S * 2))
            {
                pdu[control_mode] = control_mode_ros;			// 切换到ROS控制模式
                pdu[rc_connect_state] = rc_state_failed;		// 标记遥控器连接失败

            }
			// 如果计数器小于100秒，增加计数器
            else if (REMOTE_Count < (DELAY_COUNT_1S * 100))
            {
                REMOTE_Count++;
            }
        }
        else
        {
            pdu[control_mode] = control_mode_remote;			// 设置为遥控器控制模式
            pdu[rc_connect_state] = rc_state_success;			// 标记遥控器连接成功
            REMOTE_Count = 0;									// 重置遥控器超时计数器
        }
    }
#endif
}

/**************************************************
* 函数功能：	电机初始化开关
* 参    数：  	无
* 返 回 值：  	无
**************************************************/
bool motor_flag=false;	//ros控制电机使能
uint16_t test_a, test_b,test_x_enstate;
void MotorEnableFunction()
{
	// 遍历所有电机（motor_number表示电机总数，count为当前电机索引）
    for(uint16_t count = 0; count < pdu[motor_number]; count++)
    {
        uint16_t x_sw = motor1_state_word + count * pdu[ro_motor_gap];//电机状态字的Pdu地址
			 test_a = pdu[x_sw];
        uint16_t x_enstate = motor1_enable_state + count * pdu[ro_motor_gap];//电机使能状态的Pdu地址
			test_x_enstate = pdu[x_enstate];
        //遥控器使能或者ros下发电机使能
		if(ros_motor_en==1)
		{
			motor_flag=true;
		}
		else if(ros_motor_en==2)
		{
			motor_flag=false;
		}
        if ((Abs_int(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) || motor_flag )
        {
            //电机未使能
            if(!pdu[x_enstate])
            {
				// 提取状态字低4位（0x0F掩码），判断电机当前运行模式/状态
                switch (pdu[x_sw] & 0x0F)
                {
                    case 0x00:
                        mrd[count].d.ctrl.cw = 0x06;		// 设置控制字为0x06
                        pdu[x_enstate] = disable_state;		// 更新使能状态为禁用
                        break;
                    case 0x01:
                        mrd[count].d.ctrl.cw = 0x07;
                        pdu[x_enstate] = disable_state;
                        break;
                    case 0x03:
                        mrd[count].d.ctrl.cw = 0x0F;
                        pdu[x_enstate] = disable_state;
                        break;
                    case 0x07:
                        Motor_Enable_Flag = true;
                        pdu[x_enstate] = enable_state;
                        break;
                }
            }
        }
		 // 情况2：遥控器使能信号无效（Enable_value接近最小值）
        else if((pdu[Enable_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
        {
			// 如果当前电机已使能
            if(pdu[x_enstate])
            {
                //无需逐步下使能
                mrd[count].d.ctrl.cw = 0x00;		// 直接发送急停命令（0x00）禁用电机
                Motor_Enable_Flag = false;			// 清除全局使能标志
                pdu[x_enstate] = disable_state;		// 更新为禁用状态
                //switch (pdu[x_sw] & 0x0F)
                //{
                //    case 0x07:
                //        mrd[count].d.ctrl.cw = 0x07;
                //        pdu[x_enstate] = enable_state;
                //        break;

                //    case 0x03:
                //        mrd[count].d.ctrl.cw = 0x06;
                //        pdu[x_enstate] = enable_state;
                //        break;

                //    case 0x01:
                //        mrd[count].d.ctrl.cw = 0x00;
                //        pdu[x_enstate] = enable_state;
                //        break;

                //    case 0x00:
                //        Motor_Enable_Flag = false;
                //        pdu[x_enstate] = disable_state;
                //        break;
                //}

            }
        }
    }
}
