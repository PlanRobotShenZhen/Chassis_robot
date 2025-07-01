#include "remote.h"
#include "usartx.h"
#include "rtthread.h"
#include "485_address.h"
#include "motor_data.h"
#include "RJ_JT.h"
#include "balance.h"
#include "robot_select_init.h"

bool Sbus_Data_Parsing_Flag = false;	//��ģ������Ҫ�����־λ

/**************************************************************************
�������ܣ�ROS--Usart3 ���ֱ�����--Usart5���
��ڲ�����
����  ֵ��
**************************************************************************/
void Remote_Task(void *pvParameters)
{
    while (1)
    {
        rt_thread_delay(TIM);
        Usart3_Recv();			//����3(ROS)��������
        Usart3_Send();    		//����3(ROS)��������
        Sbus_Data_Parsing();	//������ģ����
        MotorEnableFunction();	//���ʹ��   0 -6 -7 -15  �����ֵ��л�
    }
}


/**************************************************
* �������ܣ���sbus�ź�ת��Ϊͨ��ֵ
* ��    ����Uart5_BufferΪ���յ��Ĵ�������
* �� �� ֵ����
**************************************************/
int REMOTE_Count = 0;
void Sbus_Data_Parsing(void)
{
//�����õ���SBUSЭ�飬�ο���ַ��https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
    /*SBUS��25���ֽڣ�14ms����һ�Σ�����֡����1����ʼλ+8λ����λ+1��żУ��+2��ֹͣλ��
	��DMA��Uart5_Bufferֻ����25��8λ����λ
    ��2���ֽ�~��23���ֽ�Ϊ��Ӧ16��ͨ����ÿ��ͨ��11bit��22*8=16*11��ÿ��ͨ��ȡֵ��ΧΪ0~2^11-1=0-2047������ȡֵ240-1807��
    ��λ�ȷ���ch1��11λ=data2�ĵ�3λ+data1��8λ���������ơ���24���ֽ�Ϊ״̬��flag��0��������������ȷ��1����ʧ�ܡ�
    */
	// �����ǰʹ�õ���FS-i6�ͺŵ�ң����
#if(REMOTE_TYPE == REMOTE_TYPE_FSi6)
	 // ң���������������ڼ��ң�����Ƿ�ʧȥ����
    // ��������С��100�루����DELAY_COUNT_1S��1��ļ�����λ��ʱ���Ӽ�����
    if(REMOTE_Count < DELAY_COUNT_1S * 100)
    {
        REMOTE_Count++;
    }
#endif
	// ��SBUS���ݽ�����־����λ����ʾ���µ�ң����������Ҫ����
    if (Sbus_Data_Parsing_Flag)
    {
		// ������ݽ�����־���ȴ���һ������
        Sbus_Data_Parsing_Flag = false;

        //0��ʼ�ֽ�Ϊ0F��23��־����Ϊ00��24�����ֽ�00
        //FS-i6Sң�������ֻ֧�ֵ�ǰ10��ͨ��
        // //HT-8Aң�������֧��8ͨ��
        //Uart5_Buffer[23]��������Ӧ��Ϊ00����HT-8A��ʾ03
        if (Uart5_Buffer[0] == 0x0F && Uart5_Buffer[24] == 0x00)
        {
			 // ����16��ͨ�������ݣ�ÿ��ͨ��11λ���ݣ�0x07FF��11λ�����룩
			// ͨ��1������ֽ�1���ֽ�2������
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
    else if (REMOTE_Count > 200)//���볬ʱ
    {
        pdu[rc_connect_state] = rc_state_failed;
        if (pdu[control_mode] != control_mode_ros)
        {
            pdu[control_mode] = control_mode_unknown;
        }
    }

#elif(REMOTE_TYPE == REMOTE_TYPE_HT8A)
        }
        //��ң�����Ľ��ն�δ���յ��ź�ʱһֱ������״̬����ֻ��remote��ros���ֿ���ģʽ
        //ң����Ĭ��״̬
		// ���ң�����Ƿ���Ĭ��״̬������ͨ�����м�λ�ã�ʹ��ͨ��Ϊ��Сֵ����ͣͨ��Ϊ��Сֵ��
       // pdu[Forward_value] == pdu[rc_base_value] && pdu[Turn_value] == pdu[rc_base_value] &&
       if ( pdu[Enable_value] == pdu[rc_min_value] && pdu[EmergencyStop_value] == pdu[rc_min_value])
        {
            //����Ĭ��״̬��Ϊʧȥң��������
            if (REMOTE_Count > (DELAY_COUNT_1S * 2))
            {
                pdu[control_mode] = control_mode_ros;			// �л���ROS����ģʽ
                pdu[rc_connect_state] = rc_state_failed;		// ���ң��������ʧ��

            }
			// ���������С��100�룬���Ӽ�����
            else if (REMOTE_Count < (DELAY_COUNT_1S * 100))
            {
                REMOTE_Count++;
            }
        }
        else
        {
            pdu[control_mode] = control_mode_remote;			// ����Ϊң��������ģʽ
            pdu[rc_connect_state] = rc_state_success;			// ���ң�������ӳɹ�
            REMOTE_Count = 0;									// ����ң������ʱ������
        }
    }
#endif
}

/**************************************************
* �������ܣ�	�����ʼ������
* ��    ����  	��
* �� �� ֵ��  	��
**************************************************/
bool motor_flag=false;	//ros���Ƶ��ʹ��
uint16_t test_a, test_b,test_x_enstate;
void MotorEnableFunction()
{
	// �������е����motor_number��ʾ���������countΪ��ǰ���������
    for(uint16_t count = 0; count < pdu[motor_number]; count++)
    {
        uint16_t x_sw = motor1_state_word + count * pdu[ro_motor_gap];//���״̬�ֵ�Pdu��ַ
			 test_a = pdu[x_sw];
        uint16_t x_enstate = motor1_enable_state + count * pdu[ro_motor_gap];//���ʹ��״̬��Pdu��ַ
			test_x_enstate = pdu[x_enstate];
        //ң����ʹ�ܻ���ros�·����ʹ��
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
            //���δʹ��
            if(!pdu[x_enstate])
            {
				// ��ȡ״̬�ֵ�4λ��0x0F���룩���жϵ����ǰ����ģʽ/״̬
                switch (pdu[x_sw] & 0x0F)
                {
                    case 0x00:
                        mrd[count].d.ctrl.cw = 0x06;		// ���ÿ�����Ϊ0x06
                        pdu[x_enstate] = disable_state;		// ����ʹ��״̬Ϊ����
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
		 // ���2��ң����ʹ���ź���Ч��Enable_value�ӽ���Сֵ��
        else if((pdu[Enable_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR)
        {
			// �����ǰ�����ʹ��
            if(pdu[x_enstate])
            {
                //��������ʹ��
                mrd[count].d.ctrl.cw = 0x00;		// ֱ�ӷ��ͼ�ͣ���0x00�����õ��
                Motor_Enable_Flag = false;			// ���ȫ��ʹ�ܱ�־
                pdu[x_enstate] = disable_state;		// ����Ϊ����״̬
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
