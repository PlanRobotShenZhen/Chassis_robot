#include "n32g45x.h"                    // Device header
#include "bsp.h"
#include "CircularCar.h"
#include "485_address.h"
#include <rtthread.h>
#include "mb.h"
/*******************************************************************************������*******************************************************************************/
int Volt1_LastStep = 0;
int Volt1_ThisStep = 0;
uint16_t Ultra1_cnt = 0, Ultra1_i = 0;
uint16_t Ultra2_cnt = 0, Ultra2_i = 0;
uint16_t ultrasonic1_distance, ultrasonic2_distance;
void Ultrasonic_Init(void)
{
    RCC_EnableAPB1PeriphClk(CS1_TIM_CLK | CS2_TIM_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(CS1_CLK | CS2_CLK, ENABLE);
    GPIO_InitType GPIO_InitStructure;
    /*��������ʼ��*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.Pin = CS1_TTIG_PIN;
    GPIO_InitPeripheral(CS1_TTIG_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CS2_TTIG_PIN;
    GPIO_InitPeripheral(CS2_TTIG_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(CS1_TTIG_GPIO, CS1_TTIG_PIN);
    GPIO_SetBits(CS2_TTIG_GPIO, CS2_TTIG_PIN);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.Pin = CS1_ECON_PIN;
    GPIO_InitPeripheral(CS1_ECON_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = CS2_ECON_PIN;
    GPIO_InitPeripheral(CS2_ECON_GPIO, &GPIO_InitStructure);

    TIM_ConfigInternalClk(CS1_ECON_TIM);
    TIM_ConfigInternalClk(CS2_ECON_TIM);
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP;
    TIM_TimeBaseInitStructure.Period = Ultrasonic_TIM_Period - 1;
    TIM_TimeBaseInitStructure.Prescaler = Ultrasonic_TIM_Prescaler - 1;
    TIM_TimeBaseInitStructure.RepetCnt = 0;
    TIM_InitTimeBase(CS1_ECON_TIM, &TIM_TimeBaseInitStructure);
    TIM_InitTimeBase(CS2_ECON_TIM, &TIM_TimeBaseInitStructure);
    TIM_ConfigInt(CS1_ECON_TIM, CS1_ECON_TIM_CCx, ENABLE);
    /*Ԥ�����֣������ڱ��ز����Ż�*/
    //TIM_ICInitType TIM_ICInitStructure;
    //TIM_ICInitStructure.Channel = CS1_ECON_Channel;
    //TIM_ICInitStructure.IcFilter = 0xF;
    //TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_BOTHEDGE;
    //TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    //TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    //TIM_ICInit(CS1_ECON_TIM, &TIM_ICInitStructure);
    //TIM_ICInitStructure.IcFilter = 0xF;
    //TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_RISING;
    //TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    //TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    //TIM_ICInitStructure.Channel = CS2_ECON_Channel;
    //TIM_ICInit(CS2_ECON_TIM, &TIM_ICInitStructure);

    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = CS1_ECON_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CS2_ECON_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Enable(CS1_ECON_TIM, ENABLE);
    TIM_Enable(CS2_ECON_TIM, ENABLE);

}

uint16_t Get_U1distance(void)
{
    //���س��������룬��Ч����0-6m��������ֵ��λΪmm
    uint16_t U1distance = Ultra1_cnt * 340 / 200;

    if (U1distance > 65000)
        return 65000;
    else
        return U1distance;

}

uint16_t Get_U2distance(void)
{
    uint16_t U2distance = Ultra2_cnt * 340 / 200;

    if (U2distance > 65000)
        return 65000;
    else
        return U2distance;
}

void TIM3_IRQHandler(void)
{
    //�ö�ʱ�������ʱ��10us����һ���ж�
    if (TIM_GetIntStatus(CS1_ECON_TIM, CS1_ECON_TIM_CCx) == SET)
    {
        TIM_ClrIntPendingBit(CS1_ECON_TIM, CS1_ECON_TIM_CCx);

        //��¼���������ն˵ĸߵ�ƽʱ�䣬оƬ��ƽ����
        if (GPIO_ReadInputDataBit(CS1_ECON_GPIO, CS1_ECON_PIN) == RESET)
        {
            Ultra1_i++;
        }

        if (GPIO_ReadInputDataBit(CS1_ECON_GPIO, CS1_ECON_PIN) == SET)
        {
            if (Ultra1_i != 0)
            {
                Ultra1_cnt = Ultra1_i;
            }

            Ultra1_i = 0;
        }

        if (GPIO_ReadInputDataBit(CS2_ECON_GPIO, CS2_ECON_PIN) == RESET)
        {
            Ultra2_i++;
        }

        if (GPIO_ReadInputDataBit(CS2_ECON_GPIO, CS2_ECON_PIN) == SET)
        {
            if (Ultra2_i != 0)
            {
                Ultra2_cnt = Ultra2_i;
            }

            Ultra2_i = 0;
        }

        //���벶��
        //if (GPIO_ReadInputDataBit(CS1_ECON_GPIO, CS1_ECON_TIM_CCx) == RESET)
        //{
        //    Volt1_ThisStep = 1;
        //    Ultrasonic1_cnt2 = 1;
        //}
        //else
        //{
        //    Volt1_ThisStep = 0;
        //    Ultrasonic1_cnt2 = 2;
        //}
        //if (Volt1_LastStep== 1 && Volt1_ThisStep == 0)//�½���ʱ��
        //{
        //    Ultrasonic1_cnt1 = TIM_GetCap2(CS1_ECON_TIM);
        //}
        //if (Volt1_LastStep == 0 && Volt1_ThisStep == 1)//������ʱ��
        //{
        //    Ultrasonic1_cnt2 = TIM_GetCap2(CS1_ECON_TIM);
        //    if (Ultrasonic1_cnt2 > Ultrasonic1_cnt1)
        //    {
        //        Distance1 = Ultrasonic1_cnt2 - Ultrasonic1_cnt1;
        //    }
        //    else
        //    {
        //        //cnt���
        //        Distance1 = Ultrasonic1_cnt2 - Ultrasonic1_cnt1 + Ultrasonic_TIM_Period;
        //    }
        //}

    }
}

void Ultrasonic_Start(void)
{
    int ultrasonic_count = 0;
    //���������Ͷ˽��յ�10us��ƽ�����������ͣ�оƬ��ƽ����
    GPIO_ResetBits(CS1_TTIG_GPIO, CS1_TTIG_PIN);
    GPIO_ResetBits(CS2_TTIG_GPIO, CS2_TTIG_PIN);

    for (ultrasonic_count = 0; ultrasonic_count < 600; ultrasonic_count++);

    GPIO_SetBits(CS1_TTIG_GPIO, CS1_TTIG_PIN);
    GPIO_SetBits(CS2_TTIG_GPIO, CS2_TTIG_PIN);
}

/**************************************************************************
�������ܣ�����������
��ڲ�����void
����  ֵ��void
**************************************************************************/
void Ultrasonic_task(void)
{
    //Բ�ε���Ĭ������������
    Ultrasonic_Start();
    //�����������봫�͵���λ������λmm��
    ultrasonic1_distance = Get_U1distance();
    pdu[Ultrasonic1] = ultrasonic1_distance;
    ultrasonic2_distance = Get_U2distance();
    pdu[Ultrasonic2] = ultrasonic2_distance;
}
/*******************************************************************************����ͨѶ*******************************************************************************/
void IR_Init(void)
{
    // ���ⷢ��˳�ʼ��
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(MCU_INF_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.Pin = MCU_INF_TX1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(MCU_INF_TX1_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_INF_TX1_GPIO, MCU_INF_TX1_PIN);
    // ������ն˳�ʼ��
    GPIO_InitStructure.Pin = MCU_INF_RX1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(MCU_INF_RX1_GPIO, &GPIO_InitStructure);


}

void IR_SendSignalOn(void)
{
    //�°����л���ƽ
    //GPIO_ResetBits(MCU_INF_TX1_GPIO, MCU_INF_TX1_PIN);
    GPIO_SetBits(MCU_INF_TX1_GPIO, MCU_INF_TX1_PIN);

}

void IR_SendSignalOff(void)
{
    GPIO_ResetBits(MCU_INF_TX1_GPIO, MCU_INF_TX1_PIN);
    //GPIO_SetBits(MCU_INF_TX1_GPIO, MCU_INF_TX1_PIN);
}

int IR_ReceiveSignal(void)
{
    if (GPIO_ReadInputDataBit(MCU_INF_RX1_GPIO, MCU_INF_RX1_PIN) == SET)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int start_count = 0;
int zero_count = 0;
int one_count = 0;
int start_count_complete = 0;
int zero_count_complete = 0;
int one_count_complete = 0;
int IrSendArrayindex = 0;
int IrSendArray[4];

void intToBinary(int functioncode, int* IrSendArray)
{
    // ȷ��num��0��15�ķ�Χ�ڣ���Ϊ����Ҫ����ת��Ϊ4λ��������
    functioncode &= 0x0F;

    // ��ÿһλ�Ķ�����ֵ�洢��������
    IrSendArray[0] = (functioncode & 0x08) ? 1 : 0;
    IrSendArray[1] = (functioncode & 0x04) ? 1 : 0;
    IrSendArray[2] = (functioncode & 0x02) ? 1 : 0;
    IrSendArray[3] = (functioncode & 0x01) ? 1 : 0;
}

/*���ⷢ��*/
void ir_sendzero(void)
{
    if (zero_count < 3)
    {
        IR_SendSignalOn();
    }
    else if (zero_count < 10)
    {
        IR_SendSignalOff();
    }

    zero_count++;

    if (zero_count == 10)
    {
        zero_count = 0;
        zero_count_complete = 1;
    }
}

void ir_sendone(void)
{
    if (one_count < 7)
    {
        IR_SendSignalOn();
    }
    else if (one_count < 11)
    {
        IR_SendSignalOff();
    }

    one_count++;

    if (one_count == 11)
    {
        one_count = 0;
        one_count_complete = 1;
    }
}

void ir_sendstart(void)
{
    if (start_count < 10)
    {
        IR_SendSignalOn();
    }
    else if (start_count < 14)
    {
        IR_SendSignalOff();
    }

    start_count++;

    if (start_count == 14)
    {
        start_count = 0;
        start_count_complete = 1;
    }
}

void ir_sendfunctioncode(int* IrSendArray)
{
    if (start_count_complete == 0)
    {
        ir_sendstart();
    }
    else if (start_count_complete == 1)
    {
        if (IrSendArray[IrSendArrayindex] == 0)
        {
            ir_sendzero(); // ����ir_sendzero����

            if (zero_count_complete == 1)
            {
                IrSendArrayindex++;
                zero_count_complete = 0;
            }
        }
        else if (IrSendArray[IrSendArrayindex] == 1)
        {
            ir_sendone();  // ����ir_sendone����

            if (one_count_complete == 1)
            {
                IrSendArrayindex++;
                one_count_complete = 0;
            }
        }

        if (IrSendArrayindex == 4)
        {
            IrSendArrayindex = 0;
            start_count_complete = 0;
        }
    }
}

/* �������*/
int IrReceiveArray[4];
int IrReceiveArrayIndex = 0;
int hightimeState = WAIT_RISI;
int hightime_count = 0;
int hightime_millisecond = 0;
int hightime_flag = 0;
int start_receive_complete = 0;
int CarIr_RecvData = 0;

void get_hightime(void)
{
    switch (hightimeState)
    {
        case WAIT_RISI:
            if (IR_ReceiveSignal() == 0)
            {
                hightimeState = WAIT_START;
            }

            break;

        case WAIT_START:
            if (IR_ReceiveSignal() == 1)
            {
                hightimeState = WAIT_END;
            }

            break;

        case WAIT_END:
            if (IR_ReceiveSignal() == 1)
            {
                hightime_count++;
            }
            else if (IR_ReceiveSignal() == 0)
            {
                hightimeState = WAIT_RISI;
                hightime_millisecond = hightime_count * 10;
                hightime_count = 0;
                hightime_flag = 1;
            }

            break;
    }
}

void ir_decodefunctioncode(int* IrReceiveArray)
{
    if (hightime_flag == 0)
    {
        get_hightime();
    }
    else if (hightime_flag == 1)
    {
        if (start_receive_complete == 0)
        {
            if (hightime_millisecond >= 90 && hightime_millisecond <= 110)
            {
                start_receive_complete = 1;
            }

            hightime_flag = 0;
        }
        else if (start_receive_complete == 1)
        {
            if (hightime_millisecond > 10 && hightime_millisecond < 50)
            {
                IrReceiveArray[IrReceiveArrayIndex] = 0;
                IrReceiveArrayIndex++;
            }
            else if (hightime_millisecond > 50 && hightime_millisecond < 90)
            {
                IrReceiveArray[IrReceiveArrayIndex] = 1;
                IrReceiveArrayIndex++;
            }

            if (IrReceiveArrayIndex == 4)
            {
                IrReceiveArrayIndex = 0;
                start_receive_complete = 0;
            }

            hightime_flag = 0;
        }
    }
}

int BinaryToInt(int* IrReceiveArray)
{
    return (int)((IrReceiveArray[0] << 3) | (IrReceiveArray[1] << 2) | (IrReceiveArray[2] << 1) | IrReceiveArray[3]);
}
//����ԭͨѶ��ʽ
void CarIr_SendDataFcn(uint8_t SendData)
{
    intToBinary(SendData, IrSendArray); // ��������ת��Ϊ���������ݣ�4λ��
    ir_sendfunctioncode(IrSendArray);   // ��ʼ��+�����뷢�Ͳ���
}

int CarIr_RecvDataFcn(void)
{
    ir_decodefunctioncode(IrReceiveArray);// ������յ�������
    return BinaryToInt(IrReceiveArray);// ������յ�������
}
int temp1;
/**************************************************************************
�������ܣ�����ͨѶ����
��ڲ�����
����  ֵ��
**************************************************************************/
void IrDA_task(void)
{
    static int ir_state = 0;
    temp1 = ir_state;

    switch (ir_state)
    {
        case 0://�رպ��⹦�ܣ��ȴ�����ָ��

            //���յ�������λ���ĳ��ָ��
            if (pdu[ros_chargingcommand] == 1)
            {
                ir_state = 1;
            }

        case 1://�������⹦�ܣ��ȴ�����
            IR_SendSignalOn();              // ����˷����ź�

            if (IR_ReceiveSignal() == 1)    // ������ն˽��յ��ź�
            {
                // �����Ѷ���
                IR_Init();
                pdu[ir_functioncode] = Charge_On;
                ir_state = 2;
            }

        case 2://��������ͨѶ�����͹����룬�շ�һ��ʱִ������
            if (pdu[ros_chargingcommand] == 0)
            {
                pdu[ir_functioncode] = Charge_Off;
            }

            //���͹�����
            CarIr_SendDataFcn(pdu[ir_functioncode]);
            //���չ�����
            CarIr_RecvData = CarIr_RecvDataFcn();

            if (pdu[ir_functioncode] == CarIr_RecvData)  //����շ�������һ��
            {
                switch (pdu[ir_functioncode])
                {
                    case Charge_On:  // ��ʼ���
                        //Բ�ε����°���û�е缫Ƭ�жϿ�
                        //if (GPIO_ReadOutputDataBit(YL_3_GPIO, YL_3_RxPin) == 0)
                        //{
                        //    electrodepad_on();
                        //}
                        //С����������
                        GPIO_SetBits(MCU_CHARGE_ON_GPIO, MCU_CHARGE_ON_PIN);

                        if (pdu[BatteryQuantity] >= 9950)
                        {
                            //��ص�������99.5%��Ĭ�ϳ���
                            pdu[ir_functioncode] = Charge_Off;
                        }

                        break;

                    case Charge_Off: // �����������������
                        //electrodepad_off();
                        //С���رճ���
                        GPIO_ResetBits(MCU_CHARGE_ON_GPIO, MCU_CHARGE_ON_PIN);
                        IR_SendSignalOff();
                        ir_state = 3;
                        break;

                    case Charge_Error:

                    //������ʱδ���
                    default:
                        break;
                }
            }
            else
            {
                //ir_state = 3;
            }

        case 3:
            //electrodepad_off();
            //ir_state = 2;
            break;
    }
}
/*******************************************************************************�缫Ƭ*******************************************************************************/
//Բ�ε����°���û�е缫Ƭ�жϿ�
void ElectrodePad_Init(void)
{
    //RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    //
    //GPIO_InitType GPIO_InitStructure;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.Pin = YL_3_RxPin;
    //GPIO_InitPeripheral(YL_3_GPIO, &GPIO_InitStructure);
    //
    //GPIO_ResetBits(YL_3_GPIO, YL_3_RxPin);
}

void electrodepad_on(void)
{
    // GPIO_SetBits(YL_3_GPIO, YL_3_RxPin);
}

void electrodepad_off(void)
{
    // GPIO_ResetBits(YL_3_GPIO, YL_3_RxPin);
}

void CircularCar_task(void* pvParameters)
{
    while(1)
    {
        rt_thread_delay(100);   // 10ms����һ�θ��߳�
        IrDA_task();
        Ultrasonic_task();

        if (pdu[BatteryQuantity] <= 2000)
        {
            //��ص�������99.5%��Ĭ�ϳ���
            pdu[ros_chargingcommand] = 2;
        }
        else
        {
            pdu[ros_chargingcommand] = 1;
        }
    }
}
