#include "Charger.h"
#include <rtthread.h>
#include "led.h"
#include  "485_address.h"
#include "mb.h"
extern EXIO_INPUT exio_input;
/*红外通讯发送部分参数*/
uint8_t IrDA_SendState = 1;		//0：关闭红外传感器；1：红外对接；2：红外通讯；
uint8_t SendCout = 0;			//发送计次
uint8_t SendGuide_Flag = 0;		//引导位状态
int Send_i = 3;					//发送数据指针，从高位开始发送
int BitFree = 1;				//发送通道空闲
uint8_t SendData = 0x01;		//要发送的数据
/*红外通讯接收部分参数*/
uint8_t ReceiveData;			//接收到的数据
uint8_t IrDA_AlignOK = 0;		//红外对齐，断开0，闭合1；

uint8_t LimitSwitch_OK = 0;		//限位开关，断开0，闭合1；
uint8_t CH_ON = 0;				//充电电极短路检测模块开启信号
/*接收数据部分参数*/
int Receive_i = 3; 		//接收数据指针，从高位开始发送
uint8_t guide_flag = 0;	//接收到引导位
uint16_t high_count = 0;//接收计次
int BitEnd = 0;			//接收到一个完整字

/*RGB参数*/
uint8_t ChipEndFlag = 0;
uint8_t RGB_ChangeFlag = 0;
uint8_t RGB_ArrayNum = 0;
int RGB_i = 23;//RGB指针
int bit_i = 7;//八位数据的位指针

int RGB_Array[][3] =  //普通 RGB 颜色的颜色模式
{
    {0xFF, 0x7F, 0x00},
    {0xFF, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0xFF},
    {0x00, 0x00, 0xFF},
    {0x8B, 0x00, 0xFF}
};
int RGB_Charging[][3] =  //在充电中状态下的 RGB 灯光模式。
{
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0x00, 0x00},
    {0x00, 0xFF, 0x00}
};
int RGB_Charged[][3] =  //在充电完成状态下的 RGB 灯光模式。
{
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00},
    {0x00, 0xFF, 0x00}
};
int RGB_Error[][3] =  //错误状态下的 RGB 灯光模式。
{
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
    {0xFF, 0x00, 0x00},
    {0x00, 0x00, 0x00},
};
/**************************************************************************
函数功能：MCU_INF_RX接收到来自小车的红外信号之后，开启充电桩的红外发送功能
入口参数：无
引脚信息：未接收到信号时，RX信号端低，ADCJT(PA0)高；
**************************************************************************/
void IrDA_TX_Control()
{
    //接收到红外信号
    if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO, MCU_INF_RX_PIN) == RESET)
    {
        //1启动红外传感器发送端
        MCU_INF_TX = 1;
    }
    else
    {
        MCU_INF_TX = 0;
    }
}

/**************************************************************************
函数功能：红外通讯发送数据部分
入口参数：无/要发送的数据
引脚信息：发送信号端低电平时，TX信号端高，RGBG低；
**************************************************************************/
void IrDA_Guide(void)
{
    SendCout++;

    //高电平100ms
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

    //高电平30ms
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

    //高电平70ms
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
    if (SendGuide_Flag == 0)//0:未发送引导位；1：已发送引导位；
    {
        IrDA_Guide();
    }
    else
    {
        uint8_t bit = (SendData >> Send_i) & 0x01;

        if (bit)
        {
            IrDA_Send1();

            if (BitFree)//等待发送完成
            {
                Send_i--;//指针指向低位
            }
        }
        else
        {
            IrDA_Send0();

            if (BitFree)//等待发送完成
            {
                Send_i--;//指针指向低位
            }
        }

        if (Send_i == -1)
        {
            //数据发送完成
            Send_i = 3;//指针重置到数据高位
            SendGuide_Flag = 0;//置零引导位
        }
    }
}
/**************************************************************************
函数功能：红外通讯接收端解码
入口参数：无
备注信息：1-对接正常；2-关闭充电；3-充电异常
			TX端发送解码值，并保存到PDU
**************************************************************************/
void IrDA_RX_Decode(void)
{
    ReceiveData = IrDA_ReceiveData(pdu);

    //test
    //ReceiveData = 1;
    /*红外接收端解码*/
    switch (ReceiveData)
    {
        case 0:
            break;

        case 0x01://红外对接正常
            SendData = 1;
            IrDA_SendData(SendData);
            RGB_ShowCharging();
            IrDA_AlignOK = 1;
            break;

        case 0x02://关闭充电
            SendData = 2;
            IrDA_SendData(SendData);
            RGB_ShowCharged();
            IrDA_AlignOK = 0;
            break;

        case 0x03://充电异常
            SendData = 3;
            IrDA_SendData(SendData);
            RGB_ShowError();
            IrDA_AlignOK = 0;
            break;

        default:
            break;
    }

    /*等待限位开关信号*/
    if (GPIO_ReadInputDataBit(MCU_SW_DET_GPIO, MCU_SW_DET_PIN) == RESET)
    {
        LimitSwitch_OK = 1;
    }
    else
    {
        LimitSwitch_OK = 0;
    }

    //小车与充电桩对接成功(红外对接成功+限位开关闭合)，MCU关闭MCU_CH_DET_ON
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
    // CH_ON = 1:小车与充电桩对接成功(红外通讯正常和限位开关闭合)
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
        //开启继电器
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
函数功能：初始化红外接收 GPIO 引脚
入口参数：无
返回值  ：无
**************************************************************************/
void IR_RX_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);// 启用与红外接收连接的 GPIO 引脚的时钟
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// 配置 GPIO 引脚为输入下拉模式
    GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_INF_RX_GPIO, &GPIO_InitStructure);
}

/**************************************************************************
函数功能：配置中断控制器
入口参数：无
返回值  ：无
**************************************************************************/
void NVIC_Config(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = MCU_INF_RX_TIM_IRQn;// 配置红外接收 TIM 的中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = RGB_TIM_IRQn;// 配置 RGB TIM 的中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************
函数功能：初始化红外输入捕获（IC）功能。
入口参数：无
返回值  ：无
**************************************************************************/
void IC_Init(void)
{
    /*开启时钟*/
    RCC_EnableAPB1PeriphClk(MCU_INF_RX_TIM_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(MCU_INF_RX_CLK, ENABLE);

    /*GPIO初始化*/
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_INF_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_INF_RX_GPIO, &GPIO_InitStructure);
    /*时基单元初始化*/
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1 ;    //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; //计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.Period = 65536 - 1;        //计数周期，即ARR的值，IC中设置最大，防止计数溢出
    TIM_TimeBaseInitStructure.Prescaler = 7200 - 1;      //72M/7200=10^4KHz，即0.1ms计数一次
    TIM_TimeBaseInitStructure.RepetCnt = 0;
    TIM_InitTimeBase(MCU_INF_RX_TIM, &TIM_TimeBaseInitStructure);
    /*PWMI模式初始化*/
    TIM_ICInitType TIM_ICInitStructure;
    TIM_ICInitStructure.Channel = TIM_CH_1;						//选择配置定时器通道1
    TIM_ICInitStructure.IcFilter = 0xF;							//输入滤波器参数0x0~0xF，可以过滤信号抖动
    TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_FALLING;	//极性，选择为下降沿触发捕获
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;			//捕获预分频，选择不分频，n次信号触发一次捕获
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;//输入信号交叉，选择直通，不交叉
    TIM_ConfigPwmIc(MCU_INF_RX_TIM, &TIM_ICInitStructure);		//配置TIM5的输入捕获通道
    //此函数同时会把另一个通道配置为相反的配置，实现PWMI模式
    //即滤波、分频一致；极性为下降沿触发；信号交叉；
    /*中断输出配置*/
    TIM_ClearFlag(MCU_INF_RX_TIM, TIM_FLAG_CC2);
    TIM_ConfigInt(MCU_INF_RX_TIM, TIM_INT_CC2, ENABLE);
    /*下降沿触发时置零cnt*/
    TIM_SelectInputTrig(MCU_INF_RX_TIM, TIM_TRIG_SEL_TI1FP1);
    TIM_SelectSlaveMode(MCU_INF_RX_TIM, TIM_SLAVE_MODE_RESET);
    /*TIM使能*/
    TIM_Enable(MCU_INF_RX_TIM, ENABLE);
}

/**************************************************************************
函数功能：读取红外接收数据。
入口参数：无
返回值  ：无
**************************************************************************/
uint8_t IrDA_ReceiveData(uint16_t *pdu)
{
    static uint8_t ReceiveData = 0x0;
    static uint8_t temp;								//在temp操作单字，整体赋值给RD。
    static uint16_t NoSignalCout = 0;

    if (GPIO_ReadInputDataBit(MCU_INF_RX_GPIO, MCU_INF_RX_PIN) == SET)
    {
        if (NoSignalCout >= 10000) // 如果没有接收到信号，则计数
        {
            NoSignalCout = 200;//防止溢出
        }

        NoSignalCout ++;
    }
    else
    {
        // 重置无信号计数器
        NoSignalCout = 0;
    }

    if (NoSignalCout >= 200)
    {
        // 检查无信号计数器是否达到阈值
        pdu[BatteryTemperature] = 3;//连续两秒没接收到信号，则返回充电异常信号
        return 3;
    }

    if (BitEnd == 1)
    {
        // 如果接收函数启动标志被设置
        high_count = TIM_GetCap2(MCU_INF_RX_TIM);// 获取高电平计数

        if (high_count >= 900 && high_count <= 1100)
        {
            //接收到引导位
            guide_flag = 1;
            temp = ReceiveData;
        }
        else if (guide_flag == 1)
        {
            if (high_count >= 500 && high_count < 900)
            {
                //接收到1
                //高电平70ms
                //左移i位填入变量:或运算，或1置1，或0不变
                temp  |= (1 << Receive_i);
            }
            else if (high_count >= 100 && high_count < 500)
            {
                //接收到0
                //高电平30ms
                //左移i位填入变量：与运算，与1不变，与0置0
                temp  &= ~(1 << Receive_i);
            }

            Receive_i --;								//接收指针右移

            if (Receive_i < 0)
            {
                //接收完4位数据
                Receive_i = 3;
                guide_flag = 0;
                ReceiveData = temp;						//每接收四位刷新一次数据
            }
        }
    }

    BitEnd = 0;    // 重置接收函数启动标志
    return ReceiveData;
}

/**************************************************************************
函数功能：红外接收中断处理程序
入口参数：无
返回值  ：无
**************************************************************************/
void TIM5_IRQHandler(void)
{
    if (TIM_GetIntStatus(MCU_INF_RX_TIM, TIM_INT_CC2) == SET)
    {
        // 检查 TIM5 捕获/比较通道 2 的中断状态
        BitEnd = 1;// 设置接收函数的启动标志
        TIM_ClrIntPendingBit(MCU_INF_RX_TIM, TIM_INT_CC2);// 清除 TIM5 捕获/比较通道 2 中断标志
    }
}

/**************************************************************************
函数功能：初始化按键的 GPIO 引脚,测试用
入口参数：无
返回值  ：无
**************************************************************************/
void Key_Init(void)
{
    GPIO_InitType GPIO_InitStructure;// 定义 GPIO 初始化结构体
    RCC_EnableAPB2PeriphClk(KEY_CLK, ENABLE);// 启用按键连接的 GPIO 引脚的时钟
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;// 配置 GPIO 引脚为输入下拉模式
    GPIO_InitStructure.Pin = KEY_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(KEY_GPIO, &GPIO_InitStructure);// 初始化 GPIO 引脚
    GPIO_ResetBits(KEY_GPIO, KEY_PIN);// 重置 GPIO 引脚
}

/**************************************************************************
函数功能：按键状态改变时触发 RGB 灯光变化
入口参数：无
返回值  ：无
**************************************************************************/
void Key_Change_RGB(void)
{
    if (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET)
    {
        // 检查按键是否被按下
        while (GPIO_ReadInputDataBit(KEY_GPIO, KEY_PIN) == SET);// 等待按键被释放

        RGB_ChangeFlag = 1;// 设置 RGB 灯光变化标志
        RGB_ArrayNum++;// 增加 RGB 灯光模式数组索引

        if (RGB_ArrayNum >= (sizeof(RGB_Array) / sizeof(RGB_Array[0])))
        {
            // 如果索引超过 RGB 灯光模式数组的长度，则循环到开始
            RGB_ArrayNum = 0;
        }
    }
}

/**************************************************************************
函数功能：初始化继电器的 GPIO 引脚
入口参数：无
返回值  ：无
**************************************************************************/
void Relay_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(MCU_RELAY1_CLK, ENABLE);// 启用继电器连接的 GPIO 引脚的时钟
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// 配置 GPIO 引脚为推挽输出模式
    GPIO_InitStructure.Pin = MCU_RELAY1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_RELAY1_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_RELAY1_GPIO, MCU_RELAY1_PIN);
}

/**************************************************************************
函数功能：初始化 RGB 灯光控制的 GPIO 和 TIM。
入口参数：无
返回值  ：无
**************************************************************************/
void RGB_Init(void)
{
    // 定义 GPIO 和 TIM 初始化结构体
    GPIO_InitType GPIO_InitStructure;
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    OCInitType TIM_OCInitStructure;

    // 启用 RGB 灯光连接的 GPIO 和 TIM 的时钟
    RCC_EnableAPB2PeriphClk(RGB_CLK, ENABLE);
    RCC_EnableAPB2PeriphClk(RGB_TIM_CLK, ENABLE);

    // 配置 RGB 灯光连接的 GPIO 引脚为复用推挽输出模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.Pin = RGB_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(RGB_GPIO, &GPIO_InitStructure);

    // 配置 TIM 的时基单元
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//计数器模式，选择向上计数
    TIM_TimeBaseInitStructure.Period = RGB_TIM_Period - 1;		//计数周期，即ARR的值
    TIM_TimeBaseInitStructure.Prescaler = RGB_TIM_Prescaler - 1;	//预分频器，即PSC的值
    TIM_TimeBaseInitStructure.RepetCnt = 1;						// 重复计数（用于复位模式）
    TIM_InitTimeBase(RGB_TIM, &TIM_TimeBaseInitStructure);     	//将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元

    // 配置输出比较单元
    TIM_InitOcStruct(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//输出比较模式，选择PWM模式1
    TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//输出极性，选择为高，若选择极性为低，则输出高低电平取反
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//输出使能
    TIM_OCInitStructure.Pulse = 0;								// 初始脉冲宽度（CCR 值）

    // 初始化 TIM 的输出比较通道
    TIM_InitOc4(RGB_TIM, &TIM_OCInitStructure);					//将结构体变量交给TIM_OC1Init，配置TIM1的输出比较通道1
    TIM_ConfigOc4Preload(RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);		//启用TIM预加载功能，CCR被加载时不会影响当前PWM，下个更新时间时生效。
    TIM_ConfigArPreload(RGB_TIM, ENABLE);						//启用 ARR 预加载

    // 清除 TIM 的更新中断标志并启用更新中断
    TIM_ClearFlag(RGB_TIM, TIM_INT_UPDATE);
    TIM_ConfigInt(RGB_TIM, TIM_INT_UPDATE, ENABLE);

    // 启用 TIM 和 PWM 输出
    TIM_Enable(RGB_TIM, ENABLE);
    TIM_EnableCtrlPwmOutputs(RGB_TIM, ENABLE);
}

/**************************************************************************
函数功能：设置 RGB 灯光的占空比。
入口参数：无
返回值  ：无
**************************************************************************/
void RGB_SetDuty(uint16_t Compare)
{
    TIM_SetCmp4(RGB_TIM, Compare);	// 设置 TIM 的输出比较寄存器值，以调整占空比
}

/**************************************************************************
函数功能：设定灯光的RGB值
入口参数：三原色灯的RGB值（由于中断时长原因，每传送两位数据改变一次电位）
引脚信息：PA11(YL_7)为信号输出线
**************************************************************************/
void RGB_SetValue(int G, int B, int R)
{

    int bit = 2;

    //RGB取位数据
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
            //RGB数据传输完成标志位
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
函数功能：绿灯间隔一秒闪烁
入口参数：无
返回值  ：无
**************************************************************************/
void RGB_ShowCharging(void)
{
    static int delay_i = 0;
    delay_i++;

    //每一秒切换一次颜色
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
函数功能：绿灯常亮
入口参数：无
返回值  ：无
**************************************************************************/
void RGB_ShowCharged(void)
{
    static int delay_i = 0;
    delay_i++;

    if (delay_i >= 100)
    {
        //每一秒切换一次颜色
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
函数功能：红灯间隔一秒闪烁
入口参数：无
返回值  ：无
**************************************************************************/
void RGB_ShowError(void)
{
    static int delay_i = 0;
    delay_i++;

    if (delay_i >= 100) //每一秒切换一次颜色
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
函数功能：在中断中改变定时器占空比，以实现RGB值的改变
入口参数：无（由于中断时长原因，以每两位数据中的高位为值，每发送两位进入中断改变占空比）
引脚信息：PA8(RJ_JT)为按键输入，每次按键切换一次RGB值
**************************************************************************/
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetIntStatus(RGB_TIM, TIM_INT_UPDATE) == SET)
    {
        static int Reset_i = RGB_ResetNum;
        static int RGB_chips = RGB_ChipsNum;
        TIM_ClrIntPendingBit(RGB_TIM, TIM_INT_UPDATE);

        //接收到变色信号
        if (RGB_ChangeFlag == 1)
        {
            //发送reset码
            if (Reset_i > 0)
            {
                RGB_SetDuty(RGB_Reset);
                Reset_i--;
                Reset_i--;
            }
            else if (RGB_chips > 0)
            {
                //逐次发送每颗芯片的rgb数据
                RGB_SetValue(RGB_Array[RGB_ArrayNum][0], RGB_Array[RGB_ArrayNum][1], RGB_Array[RGB_ArrayNum][2]);

                if(ChipEndFlag == 1)
                {
                    ChipEndFlag = 0;
                    RGB_chips--;
                }
            }
            //一整套RGB数据发送完毕
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
函数功能：初始化行程开关的 GPIO 引脚。
入口参数：无
返回值  ：无
**************************************************************************/
void LimitSwitch_Init(void)
{
    //默认高电平，TRAVEL_SW有信号时低电平
    // 定义 GPIO 初始化结构体
    GPIO_InitType GPIO_InitStructure;

    // 启用与行程开关连接的 GPIO 引脚的时钟
    RCC_EnableAPB2PeriphClk(MCU_SW_DET_CLK, ENABLE);

    // 配置 GPIO 引脚为输入上拉模式
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_SW_DET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 初始化 GPIO 引脚
    GPIO_InitPeripheral(MCU_SW_DET_GPIO, &GPIO_InitStructure);
}

/**************************************************************************
函数功能：初始化与充电检测相关的 GPIO 引脚。
入口参数：无
返回值  ：无
**************************************************************************/
void ChargeDetection_Init(void)
{
    // 定义 GPIO 初始化结构体
    GPIO_InitType GPIO_InitStructure;
    //默认低电平，高电平时所有检测正常
    // 初始化充电检测引脚（输入上拉模式）
    RCC_EnableAPB2PeriphClk(MCU_CH_DET_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = MCU_CH_DET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_CH_DET_GPIO, &GPIO_InitStructure);
    //默认高电平，低电平时小车与充电桩对接成功
    // 初始化充电检测控制引脚（推挽输出模式）
    RCC_EnableAPB2PeriphClk(MCU_CH_DET_ON_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = MCU_CH_DET_ON_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_CH_DET_ON_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(MCU_CH_DET_ON_GPIO, MCU_CH_DET_ON_PIN);

    //默认低电平，高电平时充电电极短接
    RCC_EnableAPB2PeriphClk(MCU_WARM_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = MCU_WARM_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(MCU_WARM_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_WARM_GPIO, MCU_WARM_PIN);
}
void Charge_IrDA(void)
{
    /*红外发送端功能切换*/
    switch (IrDA_SendState)  //0：关闭发送端；1：红外对接；2：红外通讯；
    {
        case 0:
            break;

        case 1:
            IrDA_TX_Control();

            //如果对接成功，转通讯
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

    //test:按键切换RGB颜色
    //Key_Change_RGB();
    Relay_Switch();
}
void Charge_task(void* pvParameters)
{
    while (1)
    {
        rt_thread_delay(100);   // 10ms
        //红外对接状态不完善
        Charge_IrDA();
    }
}
