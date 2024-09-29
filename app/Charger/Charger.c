#include "Charger.h"
#include <rtthread.h>
#include "led.h"
#include  "485_address.h"
#include "mb.h"
extern EXIO_INPUT exio_input;
/*����ͨѶ���Ͳ��ֲ���*/
uint8_t IrDA_SendState = 1;		//0���رպ��⴫������1������Խӣ�2������ͨѶ��
uint8_t SendCout = 0;			//���ͼƴ�
uint8_t SendGuide_Flag = 0;		//����λ״̬
int Send_i = 3;					//��������ָ�룬�Ӹ�λ��ʼ����
int BitFree = 1;				//����ͨ������
uint8_t SendData = 0x01;		//Ҫ���͵�����
/*����ͨѶ���ղ��ֲ���*/
uint8_t ReceiveData;			//���յ�������
uint8_t IrDA_AlignOK = 0;		//������룬�Ͽ�0���պ�1��

uint8_t LimitSwitch_OK = 0;		//��λ���أ��Ͽ�0���պ�1��
uint8_t CH_ON = 0;				//���缫��·���ģ�鿪���ź�
/*�������ݲ��ֲ���*/
int Receive_i = 3; 		//��������ָ�룬�Ӹ�λ��ʼ����
uint8_t guide_flag = 0;	//���յ�����λ
uint16_t high_count = 0;//���ռƴ�
int BitEnd = 0;			//���յ�һ��������

/*RGB����*/
uint8_t ChipEndFlag = 0;
uint8_t RGB_ChangeFlag = 0;
uint8_t RGB_ArrayNum = 0;
int RGB_i = 23;//RGBָ��
int bit_i = 7;//��λ���ݵ�λָ��

int RGB_Array[][3] =  //��ͨ RGB ��ɫ����ɫģʽ
{
    {0xFF, 0x7F, 0x00},
    {0xFF, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0xFF},
    {0x00, 0x00, 0xFF},
    {0x8B, 0x00, 0xFF}
};
int RGB_Charging[][3] =  //�ڳ����״̬�µ� RGB �ƹ�ģʽ��
{
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00}
};
int RGB_Charged[][3] =  //�ڳ�����״̬�µ� RGB �ƹ�ģʽ��
{
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00}
};
int RGB_Error[][3] =  //����״̬�µ� RGB �ƹ�ģʽ��
{
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
};
/**************************************************************************
�������ܣ�MCU_INF_RX���յ�����С���ĺ����ź�֮�󣬿������׮�ĺ��ⷢ�͹���
��ڲ�������
������Ϣ��δ���յ��ź�ʱ��RX�źŶ˵ͣ�ADCJT(PA0)�ߣ�
**************************************************************************/
void IrDA_TX_Control()
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

/**************************************************************************
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
/**************************************************************************
�������ܣ�����ͨѶ���ն˽���
��ڲ�������
��ע��Ϣ��1-�Խ�������2-�رճ�磻3-����쳣
			TX�˷��ͽ���ֵ�������浽PDU
**************************************************************************/
void IrDA_RX_Decode(void)
{
    ReceiveData = IrDA_ReceiveData(pdu);

    //test
    //ReceiveData = 1;
    /*������ն˽���*/
    switch (ReceiveData)
    {
        case 0:
            break;

        case 0x01://����Խ�����
            SendData = 1;
            IrDA_SendData(SendData);
            RGB_ShowCharging();
            IrDA_AlignOK = 1;
            break;

        case 0x02://�رճ��
            SendData = 2;
            IrDA_SendData(SendData);
            RGB_ShowCharged();
            IrDA_AlignOK = 0;
            break;

        case 0x03://����쳣
            SendData = 3;
            IrDA_SendData(SendData);
            RGB_ShowError();
            IrDA_AlignOK = 0;
            break;

        default:
            break;
    }

    /*�ȴ���λ�����ź�*/
    if (GPIO_ReadInputDataBit(MCU_SW_DET_GPIO, MCU_SW_DET_PIN) == RESET)
    {
        LimitSwitch_OK = 1;
    }
    else
    {
        LimitSwitch_OK = 0;
    }

    //С������׮�Խӳɹ�(����Խӳɹ�+��λ���رպ�)��MCU�ر�MCU_CH_DET_ON
    if (IrDA_AlignOK == 1 && LimitSwitch_OK == 1)
    {
        GPIO_ResetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);
    }
    else
    {
        GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);
    }
}


void Relay_Switch(void)
{
    // CH_ON = 1:С������׮�Խӳɹ�(����ͨѶ��������λ���رպ�)
    if (GPIO_ReadOutputDataBit(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN) == RESET)
    {
        CH_ON = 1;
    }
    else
    {
        CH_ON = 0;
    }

    if (GPIO_ReadInputDataBit(MCU_CH_DET_GPIO, MCU_CH_DET_PIN) == SET)
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
�������ܣ���ʼ��������� GPIO ����
��ڲ�������
����ֵ  ����
**************************************************************************/
void IR_RX_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);// ���������������ӵ� GPIO ���ŵ�ʱ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// ���� GPIO ����Ϊ��������ģʽ
    GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_INF_RX_GPIO, &GPIO_InitStructure);
}

/**************************************************************************
�������ܣ������жϿ�����
��ڲ�������
����ֵ  ����
**************************************************************************/
void NVIC_Config(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = MCU_INF_RX_TIM_IRQn;// ���ú������ TIM ���ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = RGB_TIM_IRQn;// ���� RGB TIM ���ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************
�������ܣ���ʼ���������벶��IC�����ܡ�
��ڲ�������
����ֵ  ����
**************************************************************************/
void IC_Init(void)
{
    /*����ʱ��*/
    RCC_EnableAPB1PeriphClk(MCU_INF_RX_TIM_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);

    /*GPIO��ʼ��*/
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_INF_RX_GPIO, &GPIO_InitStructure);
    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1 ;    //ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; //������ģʽ��ѡ�����ϼ���
    TIM_TimeBaseInitStructure.Period = 65536 - 1;        //�������ڣ���ARR��ֵ��IC��������󣬷�ֹ�������
    TIM_TimeBaseInitStructure.Prescaler = 7200 - 1;      //72M/7200=10^4KHz����0.1ms����һ��
    TIM_TimeBaseInitStructure.RepetCnt = 0;
    TIM_InitTimeBase(MCU_INF_RX_TIM, &TIM_TimeBaseInitStructure);
    /*PWMIģʽ��ʼ��*/
    TIM_ICInitType TIM_ICInitStructure;
    TIM_ICInitStructure.Channel = TIM_CH_1;						//ѡ�����ö�ʱ��ͨ��1
    TIM_ICInitStructure.IcFilter = 0xF;							//�����˲�������0x0~0xF�����Թ����źŶ���
    TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_FALLING;	//���ԣ�ѡ��Ϊ�½��ش�������
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;			//����Ԥ��Ƶ��ѡ�񲻷�Ƶ��n���źŴ���һ�β���
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;//�����źŽ��棬ѡ��ֱͨ��������
    TIM_ConfigPwmIc(MCU_INF_RX_TIM, &TIM_ICInitStructure);		//����TIM5�����벶��ͨ��
    //�˺���ͬʱ�����һ��ͨ������Ϊ�෴�����ã�ʵ��PWMIģʽ
    //���˲�����Ƶһ�£�����Ϊ�½��ش������źŽ��棻
    /*�ж��������*/
    TIM_ClearFlag(MCU_INF_RX_TIM, TIM_FLAG_CC2);
    TIM_ConfigInt(MCU_INF_RX_TIM, TIM_INT_CC2, ENABLE);
    /*�½��ش���ʱ����cnt*/
    TIM_SelectInputTrig(MCU_INF_RX_TIM, TIM_TRIG_SEL_TI1FP1);
    TIM_SelectSlaveMode(MCU_INF_RX_TIM, TIM_SLAVE_MODE_RESET);
    /*TIMʹ��*/
    TIM_Enable(MCU_INF_RX_TIM, ENABLE);
}

/**************************************************************************
�������ܣ���ȡ����������ݡ�
��ڲ�������
����ֵ  ����
**************************************************************************/
uint8_t IrDA_ReceiveData(uint16_t *pdu)
{
    static uint8_t ReceiveData = 0x0;
    static uint8_t temp;								//��temp�������֣����帳ֵ��RD��
    static uint16_t NoSignalCout = 0;

    if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO, MCU_INF_RX_PIN) == SET)
    {
        if (NoSignalCout >= 10000) // ���û�н��յ��źţ������
        {
            NoSignalCout = 200;//��ֹ���
        }

        NoSignalCout ++;
    }
    else
    {
        // �������źż�����
        NoSignalCout = 0;
    }

    if (NoSignalCout >= 200)
    {
        // ������źż������Ƿ�ﵽ��ֵ
        pdu[BatteryTemperature] = 3;//��������û���յ��źţ��򷵻س���쳣�ź�
        return 3;
    }

    if (BitEnd == 1)
    {
        // ������պ���������־������
        high_count = TIM_GetCap2(MCU_INF_RX_TIM);// ��ȡ�ߵ�ƽ����

        if (high_count >= 900 && high_count <= 1100)
        {
            //���յ�����λ
            guide_flag = 1;
            temp = ReceiveData;
        }
        else if (guide_flag == 1)
        {
            if (high_count >= 500 && high_count < 900)
            {
                //���յ�1
                //�ߵ�ƽ70ms
                //����iλ�������:�����㣬��1��1����0����
                temp  |= (1 << Receive_i);
            }
            else if (high_count >= 100 && high_count < 500)
            {
                //���յ�0
                //�ߵ�ƽ30ms
                //����iλ��������������㣬��1���䣬��0��0
                temp  &= ~(1 << Receive_i);
            }

            Receive_i --;								//����ָ������

            if (Receive_i < 0)
            {
                //������4λ����
                Receive_i = 3;
                guide_flag = 0;
                ReceiveData = temp;						//ÿ������λˢ��һ������
            }
        }
    }

    BitEnd = 0;    // ���ý��պ���������־
    return ReceiveData;
}

/**************************************************************************
�������ܣ���������жϴ������
��ڲ�������
����ֵ  ����
**************************************************************************/
void TIM5_IRQHandler(void)
{
    if (TIM_GetIntStatus(MCU_INF_RX_TIM, TIM_INT_CC2) == SET)
    {
        // ��� TIM5 ����/�Ƚ�ͨ�� 2 ���ж�״̬
        BitEnd = 1;// ���ý��պ�����������־
        TIM_ClrIntPendingBit(MCU_INF_RX_TIM, TIM_INT_CC2);// ��� TIM5 ����/�Ƚ�ͨ�� 2 �жϱ�־
    }
}

/**************************************************************************
�������ܣ���ʼ�������� GPIO ����,������
��ڲ�������
����ֵ  ����
**************************************************************************/
void Key_Init(void)
{
    GPIO_InitType GPIO_InitStructure;// ���� GPIO ��ʼ���ṹ��
    RCC_EnableAPB2PeriphClk(KEY_CLK, ENABLE);// ���ð������ӵ� GPIO ���ŵ�ʱ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// ���� GPIO ����Ϊ��������ģʽ
    GPIO_InitStructure.Pin = KEY_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(KEY_GPIO, &GPIO_InitStructure);// ��ʼ�� GPIO ����
    GPIO_ResetBits(KEY_GPIO, KEY_PIN);// ���� GPIO ����
}

/**************************************************************************
�������ܣ�����״̬�ı�ʱ���� RGB �ƹ�仯
��ڲ�������
����ֵ  ����
**************************************************************************/
void Key_Change_RGB(void)
{
    if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET)
    {
        // ��鰴���Ƿ񱻰���
        while (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET);// �ȴ��������ͷ�

        RGB_ChangeFlag = 1;// ���� RGB �ƹ�仯��־
        RGB_ArrayNum++;// ���� RGB �ƹ�ģʽ��������

        if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
        {
            // ����������� RGB �ƹ�ģʽ����ĳ��ȣ���ѭ������ʼ
            RGB_ArrayNum = 0;
        }
    }
}

/**************************************************************************
�������ܣ���ʼ���̵����� GPIO ����
��ڲ�������
����ֵ  ����
**************************************************************************/
void Relay_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(MCU_RELAY1_CLK, ENABLE);// ���ü̵������ӵ� GPIO ���ŵ�ʱ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// ���� GPIO ����Ϊ�������ģʽ
    GPIO_InitStructure.Pin = MCU_RELAY1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_RELAY1_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
}

/**************************************************************************
�������ܣ���ʼ�� RGB �ƹ���Ƶ� GPIO �� TIM��
��ڲ�������
����ֵ  ����
**************************************************************************/
void RGB_Init(void)
{
    // ���� GPIO �� TIM ��ʼ���ṹ��
    GPIO_InitType GPIO_InitStructure;
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    OCInitType TIM_OCInitStructure;

    // ���� RGB �ƹ����ӵ� GPIO �� TIM ��ʱ��
    RCC_EnableAPB2PeriphClk(RGB_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(RGB_TIM_CLK, ENABLE);

    // ���� RGB �ƹ����ӵ� GPIO ����Ϊ�����������ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.Pin = RGB_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(RGB_GPIO, &GPIO_InitStructure);

    // ���� TIM ��ʱ����Ԫ
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//������ģʽ��ѡ�����ϼ���
    TIM_TimeBaseInitStructure.Period = RGB_TIM_Period - 1;		//�������ڣ���ARR��ֵ
    TIM_TimeBaseInitStructure.Prescaler = RGB_TIM_Prescaler - 1;	//Ԥ��Ƶ������PSC��ֵ
    TIM_TimeBaseInitStructure.RepetCnt = 1;						// �ظ����������ڸ�λģʽ��
    TIM_InitTimeBase(RGB_TIM, &TIM_TimeBaseInitStructure);     	//���ṹ���������TIM_TimeBaseInit������TIM3��ʱ����Ԫ

    // ��������Ƚϵ�Ԫ
    TIM_InitOcStruct(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//����Ƚ�ģʽ��ѡ��PWMģʽ1
    TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//���ʹ��
    TIM_OCInitStructure.Pulse = 0;								// ��ʼ�����ȣ�CCR ֵ��

    // ��ʼ�� TIM ������Ƚ�ͨ��
    TIM_InitOc4(RGB_TIM, &TIM_OCInitStructure);					//���ṹ���������TIM_OC1Init������TIM1������Ƚ�ͨ��1
    TIM_ConfigOc4Preload(RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);		//����TIMԤ���ع��ܣ�CCR������ʱ����Ӱ�쵱ǰPWM���¸�����ʱ��ʱ��Ч��
    TIM_ConfigArPreload(RGB_TIM, ENABLE);						//���� ARR Ԥ����

    // ��� TIM �ĸ����жϱ�־�����ø����ж�
    TIM_ClearFlag(RGB_TIM, TIM_INT_UPDATE);
    TIM_ConfigInt(RGB_TIM, TIM_INT_UPDATE, ENABLE);

    // ���� TIM �� PWM ���
    TIM_Enable(RGB_TIM, ENABLE);
    TIM_EnableCtrlPwmOutputs(RGB_TIM, ENABLE);
}

/**************************************************************************
�������ܣ����� RGB �ƹ��ռ�ձȡ�
��ڲ�������
����ֵ  ����
**************************************************************************/
void RGB_SetDuty(uint16_t Compare)
{
    TIM_SetCmp4(RGB_TIM, Compare);	// ���� TIM ������ȽϼĴ���ֵ���Ե���ռ�ձ�
}

/**************************************************************************
�������ܣ��趨�ƹ��RGBֵ
��ڲ�������ԭɫ�Ƶ�RGBֵ�������ж�ʱ��ԭ��ÿ������λ���ݸı�һ�ε�λ��
������Ϣ��PA11(YL_7)Ϊ�ź������
**************************************************************************/
void RGB_SetValue(int G, int B, int R)
{

    int bit = 2;

    //RGBȡλ����
    if (RGB_i >= 16 && RGB_i < 24)
    {
        bit = (R >> bit_i) & 0x01;
    }
    else if (RGB_i >= 8 && RGB_i < 16)
    {
        bit = (G >> bit_i) & 0x01;
    }
    else if (RGB_i >= 0 && RGB_i < 8)
    {
        bit = (B >> bit_i) & 0x01;

        if (RGB_i == 1)
        {
            //RGB���ݴ�����ɱ�־λ
            ChipEndFlag = 1;
        }
    }

    bit_i--;
    RGB_i--;
    bit_i--;
    RGB_i--;

    if (bit_i < 0)
    {
        bit_i = 7;

        if (ChipEndFlag == 1)
        {
            RGB_i = 23;
        }
    }

    if (bit == 1)
    {
        RGB_SetDuty(RGB_Send1);
    }
    else if (bit == 0)
    {
        RGB_SetDuty(RGB_Send0);
    }
}

/**************************************************************************
�������ܣ��̵Ƽ��һ����˸
��ڲ�������
����ֵ  ����
**************************************************************************/
void RGB_ShowCharging(void)
{
    static int delay_i = 0;
    delay_i++;

    //ÿһ���л�һ����ɫ
    if (delay_i >= 100)
    {
        delay_i = 0;
        RGB_ChangeFlag = 1;

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RGB_Array[i][j] = RGB_Charging[i][j];
            }
        }

        RGB_ArrayNum++;

        if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
        {
            RGB_ArrayNum = 0;
        }
    }
}

/**************************************************************************
�������ܣ��̵Ƴ���
��ڲ�������
����ֵ  ����
**************************************************************************/
void RGB_ShowCharged(void)
{
    static int delay_i = 0;
    delay_i++;

    if (delay_i >= 100)
    {
        //ÿһ���л�һ����ɫ
        delay_i = 0;
        RGB_ChangeFlag = 1;

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RGB_Array[i][j] = RGB_Charged[i][j];
            }
        }

        RGB_ArrayNum++;

        if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
        {
            RGB_ArrayNum = 0;
        }
    }
}

/**************************************************************************
�������ܣ���Ƽ��һ����˸
��ڲ�������
����ֵ  ����
**************************************************************************/
void RGB_ShowError(void)
{
    static int delay_i = 0;
    delay_i++;

    if (delay_i >= 100) //ÿһ���л�һ����ɫ
    {
        delay_i = 0;
        RGB_ChangeFlag = 1;

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                RGB_Array[i][j] = RGB_Error[i][j];
            }
        }

        RGB_ArrayNum++;

        if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
        {
            RGB_ArrayNum = 0;
        }
    }
}

/**************************************************************************
�������ܣ����ж��иı䶨ʱ��ռ�ձȣ���ʵ��RGBֵ�ĸı�
��ڲ������ޣ������ж�ʱ��ԭ����ÿ��λ�����еĸ�λΪֵ��ÿ������λ�����жϸı�ռ�ձȣ�
������Ϣ��PA8(RJ_JT)Ϊ�������룬ÿ�ΰ����л�һ��RGBֵ
**************************************************************************/
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetIntStatus(RGB_TIM, TIM_INT_UPDATE) == SET)
    {
        static int Reset_i = RGB_ResetNum;
        static int RGB_chips = RGB_ChipsNum;
        TIM_ClrIntPendingBit(RGB_TIM, TIM_INT_UPDATE);

        //���յ���ɫ�ź�
        if (RGB_ChangeFlag == 1)
        {
            //����reset��
            if (Reset_i > 0)
            {
                RGB_SetDuty(RGB_Reset);
                Reset_i--;
                Reset_i--;
            }
            else if (RGB_chips > 0)
            {
                //��η���ÿ��оƬ��rgb����
                RGB_SetValue(RGB_Array[RGB_ArrayNum][0], RGB_Array[RGB_ArrayNum][1], RGB_Array[RGB_ArrayNum][2]);

                if(ChipEndFlag == 1)
                {
                    ChipEndFlag = 0;
                    RGB_chips--;
                }
            }
            //һ����RGB���ݷ������
            else if (RGB_chips == 0)
            {
                RGB_SetDuty(RGB_Reset);
                RGB_ChangeFlag = 0;
                Reset_i = RGB_ResetNum;
                RGB_chips = RGB_ChipsNum;
            }
        }
    }
}

/**************************************************************************
�������ܣ���ʼ���г̿��ص� GPIO ���š�
��ڲ�������
����ֵ  ����
**************************************************************************/
void LimitSwitch_Init(void)
{
    //Ĭ�ϸߵ�ƽ��TRAVEL_SW���ź�ʱ�͵�ƽ
    // ���� GPIO ��ʼ���ṹ��
    GPIO_InitType GPIO_InitStructure;

    // �������г̿������ӵ� GPIO ���ŵ�ʱ��
    RCC_EnableAPB2PeriphClk(MCU_SW_DET_CLK, ENABLE);

    // ���� GPIO ����Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_SW_DET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // ��ʼ�� GPIO ����
    GPIO_InitPeripheral(MCU_SW_DET_GPIO, &GPIO_InitStructure);
}

/**************************************************************************
�������ܣ���ʼ����������ص� GPIO ���š�
��ڲ�������
����ֵ  ����
**************************************************************************/
void ChargeDetection_Init(void)
{
    // ���� GPIO ��ʼ���ṹ��
    GPIO_InitType GPIO_InitStructure;
    //Ĭ�ϵ͵�ƽ���ߵ�ƽʱ���м������
    // ��ʼ����������ţ���������ģʽ��
    RCC_EnableAPB2PeriphClk(MCU_CH_DET_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_CH_DET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_CH_DET_GPIO, &GPIO_InitStructure);
    //Ĭ�ϸߵ�ƽ���͵�ƽʱС������׮�Խӳɹ�
    // ��ʼ�������������ţ��������ģʽ��
    RCC_EnableAPB2PeriphClk(MCU_CH_DET_ON_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = MCU_CH_DET_ON_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_CH_DET_ON_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);

    //Ĭ�ϵ͵�ƽ���ߵ�ƽʱ���缫�̽�
    RCC_EnableAPB2PeriphClk(MCU_WARM_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = MCU_WARM_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_WARM_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_WARM_GPIO, MCU_WARM_PIN);
}
void Charge_IrDA(void)
{
    /*���ⷢ�Ͷ˹����л�*/
    switch (IrDA_SendState)  //0���رշ��Ͷˣ�1������Խӣ�2������ͨѶ��
    {
        case 0:
            break;

        case 1:
            IrDA_TX_Control();

            //����Խӳɹ���תͨѶ
            if (MCU_INF_TX == 1)
            {
                IrDA_SendState = 2;
            }

            break;

        case 2:
            IrDA_RX_Decode();
            break;

        default:
            break;
    }

    //test:�����л�RGB��ɫ
    //Key_Change_RGB();
    Relay_Switch();
}
void Charge_task(void* pvParameters)
{
    while (1)
    {
        rt_thread_delay(100);   // 10ms
        //����Խ�״̬������
        Charge_IrDA();
    }
}
