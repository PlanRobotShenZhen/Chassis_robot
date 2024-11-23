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
    unsigned char SW : 2;//�� ��ͣ����
    unsigned char Rising : 1;
    unsigned char Descending : 1;

    unsigned char estop_soft : 1;//< ��ͣ
    unsigned char estop_soft_old : 1;//< ��ͣ
} emergency_stop;
uint8_t Torque_sdo1[1][8] 		= {0x2B, 0x15, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00}; 	//�ٶ�ģʽ�����أ�DIFFCAR��
uint8_t Torque_sdo2[1][8] 		= {0x2B, 0x15, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00}; 	//�ٶ�ģʽ�����أ�DIFFCAR��
uint8_t Torque_sdo[1][8] 		= {0x2B, 0x15, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00}; 	//�ٶ�ģʽ������
uint8_t Speed_sdo[1][8] 		= {0x2B, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};	//����ģʽ���ٶ�
uint8_t Torque_Mode_Sdo[1][8] 	= {0x2F, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};  	//��������ģʽ
uint8_t Speed_Mode_Sdo[1][8] 	= {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};  	//�����ٶ�ģʽ
static uint8_t battery_send_frame_num = 0;

/**************************************************************************
�������ܣ�AKMλ��ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void ServoPulse_Enable()
{
    //g_nDirectorΪ�ֱ�CH1�����ֵ�����ͨ��ֵ��ȡֵ��ΧΪ��784
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
                //����ƶ���λ��ȡ����ͣ����������ʹ���źţ�����һ�Σ�
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
                else if(myabs(diff_pos) < 300) //����������ٽ����
                {
                    mrd[0].d.ctrl.bit.bit4 = 0;//��ͣ
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

/*---------------------------һЩ���ܺ���--------------------------------*/
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{
    return (a < 0) ?  -a : a;
}



/**************************************************************************
�������ܣ�����������ȡ����ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
    return (insert < 0) ?  -insert : insert;
}


/**************************************************************************
�������ܣ��޷��������趨�ߵ���ֵ
��ڲ�������ֵ
����  ֵ��
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
�������ܣ������Ϣ������
��ڲ�����void
����  ֵ��void
**************************************************************************/
void BatteryInformation(void)
{
    static int bt = 0;
    static int bt_times = 100;
    short MOVE_XorZ = 0;
    short temp;

    if (uart4_recv_flag) // ��� UART4 ���ձ�־�����Ϊ�棬�����������յ�������
    {
        uart4_recv_flag = 0;

        switch (pdu[BatteryManufacturer]) // ���ݵ��������������Ϊ
        {
            case 1://< ���������Ƽ����޹�˾ 24Voltage
                if (myabs((short)pdu[target_linear_speed]) > myabs((short)pdu[target_yaw_speed])) // �Ƚ�С�������ٶȺͽ��ٶȣ�ѡ������һ��
                {
                    MOVE_XorZ = (short)pdu[target_linear_speed];
                    temp = (short)pdu[max_linear_speed];
                }
                else
                {
                    MOVE_XorZ = (short)pdu[target_yaw_speed];
                    temp = (short)pdu[max_yaw_speed];
                }

                if (MOVE_XorZ < 0)  // ����ֵ���ƶ��ٶ�
                {
                    MOVE_XorZ = - MOVE_XorZ;
                }// �����ƶ��ٶȵ���ˢ�¼��

                bt_times = 	MOVE_XorZ > (short)(temp / 1.2) ? 8 :
                            MOVE_XorZ > (short)(temp / 2) 	? 16 :
                            MOVE_XorZ > (short)(temp / 4) 	? 32 :
                            MOVE_XorZ > (short)(temp / 8) 	? 64 : 100;

                if (uart4_recv_data[0] == 0x3B) // �����յ��� UART4 ����ͷ�Ƿ���ȷ
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

            default: //48Voltage
                bt_times = 100;//< 1000msˢ��1��

                if (uart4_recv_data[0] == 0x7e)
                {
                    uint32_t tmp = 0;
                    int i = 0;
                    pdu[BatteryStatus] = 1;//< ��ض�ȡ�ɹ�
                    pdu[BatteryQuantity] = (uint16_t)uart4_recv_data[107] | (((uint16_t)uart4_recv_data[108]) << 8);//< ��ص���
                    pdu[BatteryQuantity] *= 10;
                    i = 97;
                    tmp = (uint32_t)uart4_recv_data[i++];
                    tmp |= (uint32_t)uart4_recv_data[i++] << 8;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 16;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 24;
                    pdu[BatteryVoltage] = (int16_t)((int)tmp * 0.1); //< ��ص�ѹ
                    tmp = (uint32_t)uart4_recv_data[i++];
                    tmp |= (uint32_t)uart4_recv_data[i++] << 8;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 16;
                    tmp |= (uint32_t)uart4_recv_data[i++] << 24;
                    pdu[BatteryCurrent] = (int16_t)((int)tmp * 0.1);//< ��ص���
                    pdu[BatteryTemperature] = (uint16_t)uart4_recv_data[77] | (((uint16_t)uart4_recv_data[78]) << 8);//< ����¶�
                }

                break;
        }
    }

    bt++;

    if (bt >= bt_times)
    {
        bt = 0;
        GPIO_SetBits(UARTFour_485en_GPIO, UARTFour_485enPin);	// �� UART4 485 ʹ������
        DMA_EnableChannel(UARTFour_Tx_DMA_Channel, DISABLE);    // �ر� DMA2 ͨ��5, UART4_TX
        DMA_SetCurrDataCounter(UARTFour_Tx_DMA_Channel, battery_send_frame_num);  // ���������Ĵ���ֻ����ͨ��������(DMA_CCRx��EN=0)ʱд��
        DMA_EnableChannel(UARTFour_Tx_DMA_Channel, ENABLE);    // ���� DMA2 ͨ��5, UART4_TX
    }
}
#if(CARMODE != Diff)
/**************************************************************************
�������ܣ���Դ����
��ڲ�����void
����  ֵ��void
**************************************************************************/
void PowerControl(void)
{
    static union
    {
        struct
        {
            uint16_t jdq1 : 1;//< ������Դ JDQ1_EN
            uint16_t jdq2 : 1;//< �ҵ����Դ JDQ2_EN
            uint16_t p12v : 1;//< 12V��Դ YL_7
            uint16_t p19v : 1;//< 19V��Դ YL_6
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
�������ܣ�SPI��д����
��ڲ�����void
����  ֵ��void
**************************************************************************/
void SPI1_ReadWriteByte(void)
{
    if (!SPI_ReadWriteCycle)
    {
        uint16_t retry = 0;
        GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_SET);
        pdu[exio_input_status] = exio_input.input = SPI_Master_Rx_Buffer;

        for (retry = 0; retry < 1000;) // ��ѭ�������ڵȴ�
        {
            retry++;
        }

        retry = 0;

        while (!(SPI_MASTER->STS & 1 << 1)) //�ȴ���������
        {
            retry++;

            if (retry > 2000) // ����ȴ�ʱ�䳬�� 2000�����ʾ���ִ���
            {
                pdu[car_error_messages] = spi_error;//< SPI����
                return;// ���أ��˳�����
            }
        }

        if (((pdu[control_mode] == control_mode_ros) || (pdu[control_mode] == control_mode_ipc))
                && (robot_control.bit.light_ctrl_en)) //< Ĭ�ϵƿ���Ȩ
        {
            pdu[light_control] = Receive_Data[2];
        }
        else
        {
            pdu[light_control] = exio_output.output;
        }

        SPI_MASTER->DAT = pdu[light_control];
        SPI_ReadWriteCycle = 1; // ��� SPI ��д���ڿ�ʼ
        SPI_heartbeat = 0;
    }

    if (SPI_ReadWriteCycle)  // ��� SPI ��д�����Ѿ���ʼ
    {
        SPI_heartbeat ++;

        if (SPI_heartbeat > 20) // ���������������Ͷ�д����״̬
        {
            SPI_heartbeat = 0;
            SPI_ReadWriteCycle = 0;
        }
    }
}
#endif


/**************************************************************************
�������ܣ���س�ʼ�����
��ڲ�����
����  ֵ��
**************************************************************************/
void BatteryInfoInit(void)
{
    battery_send_frame_num = 0;

    switch (pdu[BatteryManufacturer])
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
�������ܣ����Ŀ������
��ڲ�����
����  ֵ��
**************************************************************************/
uint8_t tmp1 = 0;
void Balance_task(void* pvParameters)
{


    while (1)
    {
        rt_thread_delay(50);   //< 5ms
        //���ɣ�����˵10ms�·��ٶȱȽϺ�
        /*RunningTest*/
        tmp1++;
        if (tmp1 == 100)
        {
            //����ָʾ��
            LedBlink(LED17_GPIO, LED17_PIN);//  ����˸ �����жϴ˺����Ƿ���������
            tmp1 = 0;
        }        
        SetReal_Velocity(); //��ȡ���趨���ٶȺͷ���ֵ
				
		// ������ ��������� ��ֻ��ǰ��һ�����ת��λ��ģʽ��  ������� ȫ���ٶ�ģʽ   
        Kinematic_Analysis(pdu[target_linear_speed], Move_Y, pdu[target_yaw_speed]);//�����ٶȺͽ��ٶ�ת��Ϊ���������ֵ
       
		// ���ڽ����� ��ŵ����⣬   ͨ�� �ж�2�ŵ����ģʽ(),  ����ԭ��   Բ�̵������� ��һ�϶��������� ��һ��һ
		uint16_t x_sport_mode = (pdu[car_type] == Diff_Car) ? pdu[motor1_sport_mode] : pdu[motor2_sport_mode];
				
		//Ŀǰ����Ч�Ľ�������ǽ���� �������� ������������
        ClassificationOfMotorMotionModes(x_sport_mode);	//����˶�ģʽ����
    }
}

/**************************************************
* �������ܣ���ȡ���趨���ٶȺͷ���ֵ
**************************************************/
short test_Raw = 0;
short test_Linear = 0;
void SetReal_Velocity()
{
    if (Soft_JT_Flag) //< ��ͣ
    {
        pdu[target_linear_speed] = Move_Y = pdu[target_yaw_speed] = pdu[target_angle] = Servo_pulse = 0;
        //  target_linear_speed      Ŀ�����ٶ�  X�᷽��
        //  Move_Y                   Ŀ�����ٶ�  Y�᷽��
        //  pdu[target_yaw_speed]    Ŀ�����ٶ�  Z�᷽��   ������ת
        //  pdu[target_angle]        ����ת�ǣ�ֻ�� ������ ��Ч
        //  Servo_pulse     ������ǰ�ֵ��������  ֻ�԰�������Ч�� ����ǰ��ת��
    }
    else
    {
        switch(pdu[control_mode])  //0:δ֪��1:��ģ��2:ROS��3:��λ����4:����
        {
            case control_mode_ros:
                if(ROS_RecvFlag)
                {
                    test_Linear = pdu[target_linear_speed] =0- ConvertBytesToShort(Receive_Data[3], Receive_Data[4]);	//X���ٶ�(10-3m/s)
									
                    Move_Y = ConvertBytesToShort(Receive_Data[5], Receive_Data[6]);	//Y���ٶ�(С����Y���ٶ�Ϊ0)
									
                    test_Raw = pdu[target_yaw_speed] =(0- ConvertBytesToShort(Receive_Data[7], Receive_Data[8]));	//Z���ٶ�(10-3rad/s)
									
									
									/****************** ������ ר�� 5��  ******************/
					if(pdu[car_type] == Akm_Car){
									  // ������ ר�� 
                        Z_Radian = atan(pdu[car_wheelbase] * MAGNIFIC_10000x_DOWN * pdu[target_yaw_speed] / (pdu[target_linear_speed] + EPSILON));//������ǰ��ת�ǻ���
									      // ������ ר�� 
                        Z_Radian = fmin(Z_Radian_Max, fmax(-Z_Radian_Max, Z_Radian));//ת�ǻ����޷�(��λ��0.1rad) ������
									       // ������ ר�� 
                        pdu[target_angle] = (short)(Z_Radian * RADtoANG * MAGNIFIC_10x_UP); //����ת��  ������				
									      // ������ ר�� 
                        pdu[target_yaw_speed] = (short)(tan(Z_Radian) * (pdu[target_linear_speed] + EPSILON) / (pdu[car_wheelbase] * MAGNIFIC_10000x_DOWN));	//�������ٶ�
									      // ������ ר�� 
                        Servo_pulse = (pdu[target_angle] / DEGREE_MAX) * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];
					}
									
									
                    ROS_RecvFlag = false;
                }

                break;

            case control_mode_ipc:   //qth ��λ��
				//δ����
                //X ��
                pdu[target_linear_speed] = VirtuallyLinearVelocityGet();						//���ٶȵ�λ10(-3)m/s
                 //z ��
				pdu[target_yaw_speed] = VirtuallyAngularVelocityGet(pdu[target_linear_speed]);	//���ٶȵ�λ10(-3)rad/s
                break;

            case control_mode_remote:  //��ģң����
				//X ��
                test_Linear = pdu[target_linear_speed] = LinearVelocityGet();							//���ٶȵ�λ10(-3)m/s
				//z ��
                test_Raw = pdu[target_yaw_speed] = AngularVelocityGet(pdu[target_linear_speed]);	//���ٶȵ�λ10(-3)rad/s
                break;

            case control_mode_other:  //��������
            case control_mode_unknown: //δ֪  ����
                pdu[target_linear_speed] = Move_Y = pdu[target_yaw_speed] = pdu[target_angle] = Servo_pulse = 0;
                break;

            default:
                break;
        }

        //С�����ٶ�/���ٶ�/ǰ��ת���޷�
        if(pdu[car_type] != RC_Car)
        {   
			//�޷� ��ֹ���ޣ������Сֵ  ���û� ����
            pdu[target_linear_speed] = limit_value((short)pdu[target_linear_speed], (short)pdu[max_linear_speed]);
            pdu[target_yaw_speed] = limit_value((short)pdu[target_yaw_speed], (short)pdu[max_yaw_speed]);
            pdu[target_angle] = limit_value((short)pdu[target_angle], (short)pdu[max_angle]);
        }
    }
}

/**************************************************************************
�������ܣ������ٶȺͽ��ٶ�ת��Ϊ���������ֵ
��ڲ��������ٶ����ٶ�
����  ֵ����
**************************************************************************/
short testm1;
short testm2;

void Kinematic_Analysis(short Vx, float Vy, short Vz)
{
    float common_part = LineSpeedToMotorSpeed * MAGNIFIC_1000x_DOWN;	// ��ȡ��ͬ����
    short wheel_distance_factor;										// ��ȡ���������ľ��벿��
    float correction_factor;										   // У������
    short m;
    uint16_t line_, angle_;

    switch (pdu[car_type]) //�˶�ѧ����
    {
        case Akm_Car:   //������
            wheel_distance_factor = (short)(Vz * pdu[car_tread] * MAGNIFIC_10000x_DOWN / 2.0f);
            pdu[motor2_target_rpm] = (short)((Vx - wheel_distance_factor) * common_part);	// ��λ��r/min
            pdu[motor3_target_rpm] = -(short)((Vx + wheel_distance_factor) * common_part);	// ���� motor2 �� motor3 ��Ŀ��ת��
            ServoPulse_Enable();
            break;

        case FourWheel_Car://�����������
            wheel_distance_factor = (Vz * (pdu[car_tread] + pdu[car_wheelbase]) * MAGNIFIC_10000x_DOWN / 2.0f);
            correction_factor = 0.578f;
            pdu[motor2_target_rpm] = pdu[motor1_target_rpm] = (short)((Vx - correction_factor * wheel_distance_factor) * common_part);
            pdu[motor4_target_rpm] = pdu[motor3_target_rpm] = -(short)((Vx + correction_factor * wheel_distance_factor) * common_part);
            break;

        case Diff_Car:   //Բ�β��ٶȵ���
            pdu[car_wheelbase] = 0;//ȷ��Բ�ε������Ϊ0�����ֲ����˶�ѧģ��һ��
        case TwoWheel_Car: //�������ڲ��ٶ�
        case Tank_Car:      //̹��
            wheel_distance_factor = (Vz * (pdu[car_tread] + pdu[car_wheelbase]) * MAGNIFIC_10000x_DOWN / 2.0f);
            pdu[motor1_target_rpm] = (short)((Vx - wheel_distance_factor) * common_part);
            pdu[motor2_target_rpm] = -(short)((Vx + wheel_distance_factor) * common_part);
            testm1 = pdu[motor1_target_rpm];
            testm2 = pdu[motor2_target_rpm];				
            break;

        case RC_Car:
            m = (100 + (short)pdu[rc_magnification]) * 10;
            line_ = (short)(Vx * MAGNIFIC_1000x_DOWN * MYHALF * m) + 3000;//���ٶ�
            angle_ = (short)(Vz * MAGNIFIC_1000x_DOWN * m) + 3000;//�Ƕ�
            pdu[target_linear_speed] = line_;
            pdu[target_yaw_speed] = angle_;
            RCCAR_Process(line_, angle_);//����PWM�����
            break;

        case Charger://���׮

        default:
            break;
    }
}

/**************************************************************************
�������ܣ�����˶�ģʽ����,Ŀǰ�Ǹ���ң������λ������
��ڲ�����sport_mode
�� �� ֵ����
**************************************************************************/
uint16_t test_Torque_SWB = 0;
void ClassificationOfMotorMotionModes(uint16_t sport_mode)
{
    static int mode_count = 0;
    bool torque_switch = false; //�ٶ�ģʽ�����أ��������л���־λ
    static uint16_t Torque_SWB = 0, Torque_Max = 0, Last_Gear_value = 0, Last_Torque_value = 0;
    uint8_t Speed_Max[4] =  //����ģʽ������ٶȵ�λ��200A��ַ��Ӧ�����ת��ΪU16����Ҫ��������ת��
    {
        myabs(pdu[motor2_target_rpm]) & 0xFF, (myabs(pdu[motor2_target_rpm]) >> 8) & 0xFF,
        myabs(pdu[motor3_target_rpm]) & 0xFF, (myabs(pdu[motor3_target_rpm]) >> 8) & 0xFF,
    };

    switch (sport_mode) //ģʽ�ж�
    {
        case speed_mode:
            if(pdu[GearPosition_value] - Last_Gear_value) //��Ӧң�����ϵĸ��е�������λ�����Ķ�
            {
                if (Abs_int(pdu[GearPosition_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_LOW_GEAR;//�ٶ�ģʽ��������ص�λ
                }
                else if (Abs_int(pdu[GearPosition_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_MIDDLE_GEAR;
                }
                else if (Abs_int(pdu[GearPosition_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)
                {
                    Torque_SWB = SWB_HIGH_GEAR;  //ͨ��SDO  �����������  �����ֵ�   ֻ�������У�����û�д�ѡ�
                }
								test_Torque_SWB = Torque_SWB;
                torque_switch = true;
            }

#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)			
			//FS-i6s��ߵ���ť���������ֳɾŵ�
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
            if (pdu[Torque_value] - Last_Torque_value) //��Ӧң�����ϵĸ��е��������ص�λ�����Ķ�
            {
                if (Abs_int(pdu[Torque_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_MIN;//�ٶ�ģʽ����С���ص�λ
                }
                else if (Abs_int(pdu[Torque_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_BASE;
                }
                else if (Abs_int(pdu[Torque_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR)
                {
                    pdu[torque_cofficient] = TORQUE_COEFFICIENT_MAX; //ͨ��SDO  �����������  �����ֵ�   ֻ�������У�����û�д�ѡ�
                }
                torque_switch = true;
            }
#endif						
			//����ֵ����Ϊ�Ա�
            Last_Gear_value = pdu[GearPosition_value];
            Last_Torque_value = pdu[Torque_value];

						
			//sdo  �ѵ��� ֵ ���ͳ�ȥ
            if(torque_switch)
            {
                Odom_distance_mm = 0; //��̼����㡣����
                Torque_Max = pdu[torque_cofficient] * Torque_SWB;	//2015h����������ΧΪ0-300
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
             //  Ŀǰֻ�Ǽ򵥲��Թ���û���ٶ�ģʽ���ã����Դ˷�����ʱ����
        case torque_mode:
            switch (pdu[car_type])
            {
                case Akm_Car:
                    if((mode_count == 1) && (pdu[Forward_value] != pdu[rc_base_value])) //��ֹͣ״̬���л���ת��ģʽ������ɲ��
                    {
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        mode_count = 0;
                    }

                    if((pdu[Forward_value] == pdu[rc_base_value]) && (myabs(pdu[motor2_rpm_feedback]) < 2)) //ֹͣ�����л����ٶ�ģʽ����ɲ��
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
                    pdu[motor2_target_torque] = pdu[motor2_target_rpm] * MAGNIFIC_10x_UP; //����10����ΪZLAC��Ŀ��ת����Ŀ���ٶ�ȡֵ��Χ����10��
                    pdu[motor3_target_torque] = pdu[motor3_target_rpm] * MAGNIFIC_10x_UP;
                    break;

                case FourWheel_Car:
                    if((mode_count == 1) && (pdu[Forward_value] != pdu[rc_base_value])) //��ֹͣ״̬���л���ת��ģʽ������ɲ��
                    {
                        Add_Sdo_Linked_List(pdu[motor1_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor2_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor3_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        Add_Sdo_Linked_List(pdu[motor4_CAN_id], Torque_Mode_Sdo, sizeof(Torque_Mode_Sdo) / sizeof(Torque_Mode_Sdo[0]));
                        mode_count = 0;
                    }

                    if((pdu[Forward_value] == pdu[rc_base_value]) && (myabs(pdu[motor2_rpm_feedback]) < 2)) //ֹͣ�����л����ٶ�ģʽ����ɲ��
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
                    pdu[motor1_target_torque] = pdu[motor1_target_rpm] * MAGNIFIC_10x_UP; //����10����ΪZLAC��Ŀ��ת����Ŀ���ٶ�ȡֵ��Χ����10��
                    pdu[motor2_target_torque] = pdu[motor2_target_rpm] * MAGNIFIC_10x_UP;
                    pdu[motor3_target_torque] = pdu[motor3_target_rpm] * MAGNIFIC_10x_UP;
                    pdu[motor4_target_torque] = pdu[motor4_target_rpm] * MAGNIFIC_10x_UP;
                    break;

                case TwoWheel_Car://��������ģʽ
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
* �������ܣ��õ����ٶ�
* �� �� ֵ���������ٶ�
**************************************************/
//  �����Ӧң������ͨ��ֵ
short LinearVelocityGet(void)
{
    uint16_t VelocityCoefficient = (myabs(pdu[GearPosition_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR) ? pdu[linear_low] : //���ٶ�ϵ��
                (myabs(pdu[GearPosition_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR) ? pdu[linear_middle] : pdu[linear_high];
    float VelocityTemp = pdu[robot_forward_direction] * (pdu[Forward_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference];

    switch(pdu[car_type])
    {
        case Akm_Car:		// ������
        case FourWheel_Car:	// �������������
        case Diff_Car:		// ���ٳ�(Բ�ε���)
        case TwoWheel_Car: 	// ���ڲ��ٶ�����
        case Tank_Car:   	// ̹�˳�
        case RC_Car:   		// RC��
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?//ʹ�ܲ���
                   (short)(VelocityCoefficient * VelocityTemp) : 0;

        default:
            return 0;
    }
}

/**************************************************
* �������ܣ��õ����ٶ�
* �� �� ֵ�����ؽ��ٶ�
**************************************************/

//  �����Ӧң������ͨ��ֵ
short AngularVelocityGet(short linearValue)
{
    uint16_t YawCoefficient = (pdu[GearPosition_value] == pdu[rc_min_value]) ? pdu[angular_low] : //���ٶ�ϵ��
                              (pdu[GearPosition_value] == pdu[rc_base_value]) ? pdu[angular_middle] : pdu[angular_high];
    float YawTemp = -(pdu[robot_forward_direction] * (pdu[Turn_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference]); //���ٶ���ת��ͨ��ֵ�������෴����һ��(-1,1)

    switch(pdu[car_type])
    {
        case Akm_Car:		// ������
            pdu[target_angle] = round(YawTemp * DEGREE_MAX); //ǰ��ת��
            Servo_pulse = YawTemp * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];//����
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(linearValue * YawTemp / AKM_RAW_FACTOR) : 0; // Z����ٶ�

        case Diff_Car:		// ���ٳ�(Բ�ε���)
        case FourWheel_Car:	// �������������
        case TwoWheel_Car: 	// ���ڲ��ٶ�����
        case Tank_Car :   	// ̹�˳�
            return (myabs(pdu[Enable_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(YawCoefficient * YawTemp / PI * 4) : 0; // Z����ٶ�
            //ϵ������

        default:
            return 0;
    }
}

/**************************************************
* �������ܣ��õ����ٶ�
* �� �� ֵ�����ؽ��ٶ�
**************************************************/

// ����Ϊ��λ���������ֱ�ͨ��ֵ

short VirtuallyLinearVelocityGet(void)
{
    uint16_t VelocityCoefficient = (pdu[virtually_rc_ch6_value] == pdu[rc_min_value]) ? pdu[linear_low] : //���ٶ�ϵ��
                                   (pdu[virtually_rc_ch6_value] == pdu[rc_base_value]) ? pdu[linear_middle] : pdu[linear_high];
    float VelocityTemp = pdu[robot_forward_direction] * (pdu[virtually_rc_ch3_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference];

    switch(pdu[car_type])
    {
        case Akm_Car:		// ������
        case FourWheel_Car:	// �������������
        case Diff_Car:		// ���ٳ�(Բ�ε���)
        case TwoWheel_Car: 	// ���ڲ��ٶ�����
        case Tank_Car:   	// ̹�˳�
        case RC_Car:   		// RC��
            return (myabs(pdu[virtually_rc_ch7_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                   (short)(VelocityCoefficient * VelocityTemp) : 0;

        default:
            return 0;
    }
}

/**************************************************
* �������ܣ��õ����ٶ�
* �� �� ֵ�����ؽ��ٶ�
**************************************************/

// ����Ϊ��λ���������ֱ�ͨ��ֵ
short VirtuallyAngularVelocityGet(short linearValue)
{
    uint16_t YawCoefficient = (pdu[virtually_rc_ch6_value] == pdu[rc_min_value]) ? pdu[angular_low] : //���ٶ�ϵ��
                              (pdu[virtually_rc_ch6_value] == pdu[rc_base_value]) ? pdu[angular_middle] : pdu[angular_high];
    float YawTemp = -(pdu[robot_forward_direction] * (pdu[virtually_rc_ch1_value] - pdu[rc_base_value]) / (float)pdu[rc_gears_difference]); //���ٶ���ת��ͨ��ֵ�������෴

    switch(pdu[car_type])
    {
        case Akm_Car:		// ������
            pdu[target_angle] = round(YawTemp * DEGREE_MAX); //ǰ��ת��
            Servo_pulse = YawTemp * MYHALF * ENCODER_LINES * CORRECTION_FACTOR * pdu[motor1_reduction_ratio];//����
            return (short)(linearValue * YawTemp / AKM_RAW_FACTOR); // Z����ٶ�
            break;
        case Diff_Car:		// ���ٳ�(Բ�ε���)
        case FourWheel_Car:	// �������������
        case TwoWheel_Car: 	// ���ڲ��ٶ�����
        case Tank_Car:   	// ̹�˳�
            return (myabs(pdu[virtually_rc_ch7_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR) ?
                (short)(YawCoefficient * YawTemp / PI * 4) : 0; // Z����ٶ�
            break;
        default:
            return 0;
    }
}



/**************************************************************************
�������ܣ����׮��ʼ��
��ڲ�����
����  ֵ��
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

