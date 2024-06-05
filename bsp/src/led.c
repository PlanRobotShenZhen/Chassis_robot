/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file led.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <rtthread.h>
#include <n32g45x.h>
#include "led.h"
#include "485_address.h"
#include "robot_select_init.h"
#include "mb.h"
#include "motor_data.h"
#include "motor.h"
#include "balance.h"
#include "bsp.h"
#include "RJ_JT.h"

/**
 * @brief  Configures LED GPIO.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
    else if (GPIOx == GPIOE)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE);
    }
    else if (GPIOx == GPIOF)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
    }
    else
    {
        if (GPIOx == GPIOG)
        {
            RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOG, ENABLE);
        }
    }

    /* Configure the GPIO pin */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_InitStructure.Pin        = Pin;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitPeripheral(GPIOx, &GPIO_InitStructure);
    }
}
#if(PLAN_CONTROL_BOARD_V==11||PLAN_CONTROL_BOARD_V==12)
void ExioInit(void)
{
    SPI_InitType SPI_InitStructure;
    NVIC_InitType NVIC_InitStruct;
    GPIO_InitType GPIO_InitStructure;
    /* Enable SPI_MASTER clock and GPIO clock for SPI_MASTER and SPI_SLAVE */
    RCC_EnableAPB2PeriphClk(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK, ENABLE);
    /* Configure SPI_MASTER pins: SCK  MOSI MISO*/
    GPIO_InitStructure.Pin = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI | SPI_MASTER_PIN_MISO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = SPI_MASTER_PIN_NSS;
    GPIO_InitPeripheral(SPI_MASTER_GPIO, &GPIO_InitStructure);
    GPIO_WriteBit(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS, Bit_RESET);

    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.NSS = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_64;
    SPI_InitStructure.FirstBit = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    SPI_I2S_EnableInt(SPI1, SPI_I2S_INT_RNE, ENABLE);
    SPI_Enable(SPI1, ENABLE);
}

#endif

void UltrasonicSetEnable(int id,uint8_t en)
{
    NVIC_InitType NVIC_InitStructure;
    IRQn_Type irq_type = id == 1 ? EXTI4_IRQn : EXTI9_5_IRQn;
    FunctionalState enable = en == 1 ? ENABLE : DISABLE;
    NVIC_InitStructure.NVIC_IRQChannel = irq_type;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = enable;
    NVIC_Init(&NVIC_InitStructure);
}


/**************************************************
* �������ܣ�����gpio��ʼ����������������LED���̵�����Ԥ��������š���ͣ
* ��    ����  ��
* �� �� ֵ��  ��
**************************************************/
void GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    //NVIC_InitType NVIC_InitStruct;
    //EXTI_InitType EXTI_InitStructure;
#if(PLAN_CONTROL_BOARD_V==10)
    RCC_EnableAPB2PeriphClk(LED1_PORT_RCC | JDQ_PORT_RCC, ENABLE);

    GPIO_InitStructure.Pin = LED_R | LED_G | LED_B | LED_Battery | RUN1 | RUN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED1_PORT, LED_Battery);
    GPIO_SetBits(LED1_PORT, LED_R | LED_G | LED_B);

    GPIO_InitStructure.Pin = JDQ1_PIN | JDQ2_PIN;
    GPIO_InitPeripheral(JDQ_PORT, &GPIO_InitStructure);
    GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
    rt_thread_delay(20000);   //< 2s
    GPIO_SetBits(JDQ_PORT, JDQ2_PIN);
    rt_thread_delay(5000);   //< 500ms
#elif(PLAN_CONTROL_BOARD_V==11||PLAN_CONTROL_BOARD_V==12)

    RCC_EnableAPB2PeriphClk(LED1_PORT_RCC| LED2_PORT_RCC | JDQ_PORT_RCC| CS1_Ttig_RCC, ENABLE);
    GPIO_InitStructure.Pin = RUN1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = RUN2;
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = JDQ1_PIN | JDQ2_PIN;
    GPIO_InitPeripheral(JDQ_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = YL_6_Pin | YL_7_Pin;
    GPIO_InitPeripheral(YL_6_GPIO, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = RJ_JT_Pin;
    GPIO_InitPeripheral(RJ_JT_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(JDQ_PORT, JDQ1_PIN);
    GPIO_SetBits(YL_6_GPIO, YL_6_Pin);//< 
    GPIO_SetBits(YL_7_GPIO, YL_7_Pin);//< 
    GPIO_SetBits(RJ_JT_GPIO, RJ_JT_Pin);//< ��ʼ��Ϊ0
    rt_thread_delay(5000);   //< 500ms
    GPIO_SetBits(JDQ_PORT, JDQ2_PIN);

#elif(PLAN_CONTROL_BOARD_V==13)

    RCC_EnableAPB2PeriphClk(LED17_CLK | ESTOP_SOFT_CLK | MCU_CHARGE_ON_CLK | MCU_19VARM_PWR_ON_CLK |
        MCU_24VARM_PWR_ON_CLK | MCU_MT_PWR_ON_CLK | UNESTOP_SW_IN_CLK | ESTOP_SW_IN_CLK, ENABLE);

    GPIO_InitStructure.Pin = LED17_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED17_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(LED17_GPIO, LED17_PIN);

    /*��ͣ�����ʼ��*/
    GPIO_InitStructure.Pin = ESTOP_SOFT_PIN;
    GPIO_InitPeripheral(ESTOP_SOFT_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(ESTOP_SOFT_GPIO, ESTOP_SOFT_PIN);

    /*�Զ�������*/
    GPIO_InitStructure.Pin = MCU_CHARGE_ON_PIN;
    GPIO_InitPeripheral(MCU_CHARGE_ON_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_CHARGE_ON_GPIO, MCU_CHARGE_ON_PIN);

    /*19V��Դʹ�ܿ���*/
    GPIO_InitStructure.Pin = MCU_19VARM_PWR_ON_PIN;
    GPIO_InitPeripheral(MCU_19VARM_PWR_ON_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_19VARM_PWR_ON_GPIO, MCU_19VARM_PWR_ON_PIN);

    /*24V��Դʹ�ܿ���*/
    GPIO_InitStructure.Pin = MCU_24VARM_PWR_ON_PIN;
    GPIO_InitPeripheral(MCU_24VARM_PWR_ON_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_24VARM_PWR_ON_GPIO, MCU_24VARM_PWR_ON_PIN);

    /*�����Դʹ�ܿ���*/
    GPIO_InitStructure.Pin = MCU_MT_PWR_ON_PIN;
    GPIO_InitPeripheral(MCU_MT_PWR_ON_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(MCU_MT_PWR_ON_GPIO, MCU_MT_PWR_ON_PIN);

    /*�ͷż�ͣ���ؼ�⣨�Ⱨբ��*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.Pin = UNESTOP_SW_IN_PIN;
    GPIO_InitPeripheral(UNESTOP_SW_IN_GPIO, &GPIO_InitStructure);

    /*��ͣ���ؼ��*/
    GPIO_InitStructure.Pin = ESTOP_SW_IN_PIN;
    GPIO_InitPeripheral(ESTOP_SW_IN_GPIO, &GPIO_InitStructure);

    PWM_LED_Init();
    rt_thread_delay(5000);   //< 2s
#endif
}

/**
 * @brief  Toggles the selected Led.
 * @param GPIOx x can be A to G to select the GPIO port.
 * @param Pin This parameter can be GPIO_PIN_0~GPIO_PIN_15.
 */
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**************************************************************************
�������ܣ�LED����˸����
��ڲ������� 
����  ֵ����
**************************************************************************/
void Led_task(void *pvParameters)
{
    while(1){                    
			rt_thread_delay(200);   //< 20ms			
			Car_Light();
			JT_Light();
    }	
}  

/**************************************************************************
�������ܣ�����״̬���Ƴ���
��ڲ������� 
����  ֵ����
**************************************************************************/
void Car_Light(void)
{
    if(!pdu[Power_board_version]){//�ɵ�Դ��        
        if((short)pdu[target_yaw_speed] > 3 && !Soft_JT_Flag){//��ת
            Right = 0;	
            Left = ~Left;
            rt_thread_delay(10000);	
        }else if((short)pdu[target_yaw_speed] < -3 && !Soft_JT_Flag){//��ת
            Left = 0;	
            Right = ~Right;
            rt_thread_delay(10000);	
        }else{//SWC
            if(myabs(pdu[rc_ch8_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR){
                Left = 0;
                Right = 0;
            }else if(myabs(pdu[rc_ch8_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR){
                Left = 1;
                Right = 1;	
            }else{
                Right = ~Right;
                Left = ~Left;	
                rt_thread_delay(5000);	
                Right = ~Right;
                Left = ~Left;	
                rt_thread_delay(5000);	
            }
        }
    }
    else if(pdu[Power_board_version]){//�µ�Դ��       
        switch(pdu[car_running_state]){
            case car_emergency_stop://��ͣ
                light_time.t_cnt_Light_ALL ++;
                if((exio_output.output != 0xFC) | (exio_output.output != 0x04)){//���õƹ�           
                    exio_output.output = 0xFC; //RGB��GB����
                }
                if(light_time.t_cnt_Light_ALL >= 25){   //500ms           
                    exio_output.output ^= 0xF8; //��RGB���⣬�������˸
                    light_time.t_cnt_Light_ALL = 0;
                }  
                break;
            case car_error://����
                exio_output.output = 0xFC;  //����Լ����Ƴ���            
                break;
            case car_standby://����
            case car_running://����
#if(CARMODE != Diff)	
                exio_output.bit.RGB_R = 1;      //RGB�̵���
                exio_output.bit.Front_Red = 0;      //RGB�����
								BatteryThresholdAlarm();
                if((short)pdu[target_yaw_speed] > 0){//��ת 
										light_time.t_cnt_RR_White ++;
									exio_output.bit.LF_White = exio_output.bit.LR_White = 0;
									if(light_time.t_cnt_RR_White >= 25){//500ms
											exio_output.bit.RR_White = exio_output.bit.RF_White = ~exio_output.bit.RF_White;								                                     											
                        light_time.t_cnt_RR_White = 0;
                    }     	
                }else if((short)pdu[target_yaw_speed] < 0){//��ת
										light_time.t_cnt_LR_White ++;								
                    exio_output.bit.RR_White = exio_output.bit.RF_White = 0;                    
                    if(light_time.t_cnt_LR_White >= 25){//500ms
                        exio_output.bit.LF_White = exio_output.bit.LR_White = ~exio_output.bit.LR_White;							
                        light_time.t_cnt_LR_White = 0;
                    }     	
                }else if((short)pdu[target_linear_speed] < 0){//����									
                    exio_output.output |= 0xA0;//�󳵵�ȫ��
					                    
                }else{
								
									if(myabs(pdu[rc_ch8_value] - pdu[rc_min_value]) < CHANNEL_VALUE_ERROR){//����
                        exio_output.output &= 0x0F; //��ȫ�𣨳�RGB����������
                        light_time.t_cnt_Light_ALL = 0;
                    }else if(myabs(pdu[rc_ch8_value] - pdu[rc_base_value]) < CHANNEL_VALUE_ERROR){//����
                        exio_output.output |= 0xF0; //��ȫ������RGB����������
                        light_time.t_cnt_Light_ALL = 0;
                    }else if(myabs(pdu[rc_ch8_value] - pdu[rc_max_value]) < CHANNEL_VALUE_ERROR){//��˸
                        light_time.t_cnt_Light_ALL ++;
                        if(light_time.t_cnt_Light_ALL >= 25){//500ms
                            exio_output.output ^= 0xF0; //��RGB���⣬�������˸
                            light_time.t_cnt_Light_ALL = 0;
                        }                
                    }
		
                }
                break;
#else               

                light_time.t_cnt_LF_White ++;//��ͬһ����������֤ͬƵ�ʡ�
                if (light_time.t_cnt_LF_White >= 25)
                {//500ms
                    light_time.t_cnt_LF_White = 0;
                    if ((short)pdu[target_linear_speed] > 0) 
                    {//����                       
                        LedBlink(MCU_LED_LEFT_GPIO, MCU_LED_LEFT_PIN);
                        LedBlink(MCU_LED_RIGHT_GPIO, MCU_LED_RIGHT_PIN);
                    }
                    else if ((short)pdu[target_yaw_speed] > 0)
                    {//��ת 
                        LedBlink(MCU_LED_LEFT_GPIO, MCU_LED_LEFT_PIN);
                    }
                    else if ((short)pdu[target_yaw_speed] < 0)
                    {//��ת 
                        LedBlink(MCU_LED_RIGHT_GPIO, MCU_LED_RIGHT_PIN);
                    }
                    else
                    {
                        GPIO_ResetBits(MCU_LED_LEFT_GPIO, MCU_LED_LEFT_PIN);
                        GPIO_ResetBits(MCU_LED_RIGHT_GPIO, MCU_LED_RIGHT_PIN);
                    }
                }	
#endif								
        }                     
    }
}

/**************************************************************************
�������ܣ������ֵ����
��ڲ�����void
����  ֵ��void
**************************************************************************/
void BatteryThresholdAlarm(void)
{
	u8 BuzzerTimes;         //�������
	static u8 NowTimes = 0; //��ǰ�������
    NowTimes ++;
	if(pdu[BatteryQuantity] < pdu[Low_battery_threshold] * 100){//��ص������ڵ���ֵ
        BuzzerTimes = 15;   //����300ms
    }else if(pdu[BatteryQuantity] < pdu[Middle_battery_threshold] * 100){//��ص�����������ֵ
        BuzzerTimes = 30;   //����600ms
    }else{
        BuzzerTimes = 0;    //������
        exio_output.bit.RGB_R = 0;//����������
    }
    if((NowTimes >= BuzzerTimes) && (BuzzerTimes)){
        NowTimes = 0;
        exio_output.bit.RGB_R = ~exio_output.bit.RGB_R; // ��������
    }
}

/**************************************************************************
�������ܣ����Ƽ�ͣʱ�򳵵���˸
��ڲ������� 
����  ֵ����
**************************************************************************/
void JT_Light(void)
{
    if(!pdu[Power_board_version]){
        u8 YL7_Bit = GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_12);
        if(YL7_Bit){	
            Left = 0;
            Right = 0;
            Right = ~Right;
            Left = ~Left;	
            rt_thread_delay(5000);	
            Right = ~Right;
            Left = ~Left;	
            rt_thread_delay(5000);		
        }
    }else if(pdu[Power_board_version]){//�µ�Դ��     
#if (CARMODE != Diff)
			if(Soft_JT_Flag){
					exio_output.output = 0xFC;  //����Լ����Ƴ���

        }else{
            exio_output.output = 0x00;  //��ȫ�𣨳�RGB����������
        }
#else
				if(Soft_JT_Flag)
				{
					GPIO_SetBits(MCU_LED_LEFT_GPIO, MCU_LED_LEFT_PIN);
					GPIO_SetBits(MCU_LED_RIGHT_GPIO, MCU_LED_RIGHT_PIN);  //����Լ����Ƴ���

        }
	
#endif			
    }
}
/*PWM���͵ƹ��ʼ��*/
void PWM_LED_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    TIM_TimeBaseInitType TIM_TimeBaseInitStructure;
    OCInitType TIM_OCInitStructure;
    /*����ʱ��*/
    RCC_EnableAPB2PeriphClk(MCU_RGB_CLK | MCU_LED_CLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB1PeriphClk(MCU_RGB_TIM_CLK, ENABLE);
    /*GPIO��ʼ��*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    GPIO_InitStructure.Pin = MCU_RGB_RED_PIN;
    GPIO_InitPeripheral(MCU_RGB_RED_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = MCU_RGB_GREEN_PIN;
    GPIO_InitPeripheral(MCU_RGB_GREEN_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = MCU_RGB_BLUE_PIN;
    GPIO_InitPeripheral(MCU_RGB_BLUE_GPIO, &GPIO_InitStructure);
    //ת����ض���TIM���ų�ͻ������GPIO����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.Pin = MCU_LED_LEFT_PIN;
    GPIO_InitPeripheral(MCU_LED_LEFT_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_LED_LEFT_GPIO, MCU_LED_LEFT_PIN);

    GPIO_InitStructure.Pin = MCU_LED_RIGHT_PIN;
    GPIO_InitPeripheral(MCU_LED_RIGHT_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(MCU_LED_RIGHT_GPIO, MCU_LED_RIGHT_PIN);
    /*ʱ����Ԫ��ʼ��*/
    TIM_TimeBaseInitStructure.ClkDiv = TIM_CLK_DIV1;     		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
    TIM_TimeBaseInitStructure.Prescaler = MCU_RGB_TIM_Prescaler - 1;	//Ԥ��Ƶ������PSC��ֵ
    TIM_TimeBaseInitStructure.RepetCnt = 1;
    TIM_TimeBaseInitStructure.CntMode = TIM_CNT_MODE_UP; 		//������ģʽ��ѡ�����ϼ���
    TIM_TimeBaseInitStructure.Period = MCU_RGB_TIM_Period - 1;		//�������ڣ���ARR��ֵ
    TIM_InitTimeBase(MCU_RGB_TIM, &TIM_TimeBaseInitStructure);     	//���ṹ���������TIM_TimeBaseInit������TIM3��ʱ����Ԫ

    //TIM_TimeBaseInitStructure.CntMode = MCU_LED_TIM_Period;
    //TIM_TimeBaseInitStructure.Period = MCU_LED_TIM_Prescaler - 1;
    //TIM_InitTimeBase(MCU_LED_TIM, &TIM_TimeBaseInitStructure);

    /*����Ƚϳ�ʼ��*/
    TIM_InitOcStruct(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;				//����Ƚ�ģʽ��ѡ��PWMģʽ1
    TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;		//������ԣ�ѡ��Ϊ�ߣ���ѡ����Ϊ�ͣ�������ߵ͵�ƽȡ��
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;	//���ʹ��
    TIM_OCInitStructure.Pulse = 0;								//��ʼ��CCRֵ

    MCU_RGB_RED_TIM_CHx_Init(MCU_RGB_TIM, &TIM_OCInitStructure);       //���ṹ���������TIM_OC1Init������TIM1������Ƚ�ͨ��1
    MCU_RGB_RED_TIM_CHx_Preload(MCU_RGB_TIM, TIM_OC_PRE_LOAD_ENABLE); //����TIMԤ���ع��ܣ�CCR������ʱ����Ӱ�쵱ǰPWM���¸�����ʱ��ʱ��Ч��

    MCU_RGB_GREEN_TIM_CHx_Init(MCU_RGB_TIM, &TIM_OCInitStructure);
    MCU_RGB_GREEN_TIM_CHx_Preload(MCU_RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);

    MCU_RGB_BLUE_TIM_CHx_Init(MCU_RGB_TIM, &TIM_OCInitStructure);
    MCU_RGB_BLUE_TIM_CHx_Preload(MCU_RGB_TIM, TIM_OC_PRE_LOAD_ENABLE);

    //MCU_LED_LEFT_TIM_CHx_Init(MCU_LED_TIM, &TIM_OCInitStructure);
    //MCU_LED_LEFT_TIM_CHx_Preload(MCU_LED_TIM, TIM_OC_PRE_LOAD_ENABLE);

    //MCU_LED_RIGHT_TIM_CHx_Init(MCU_LED_TIM, &TIM_OCInitStructure);
    //MCU_LED_RIGHT_TIM_CHx_Preload(MCU_LED_TIM, TIM_OC_PRE_LOAD_ENABLE);

    TIM_ConfigArPreload(MCU_RGB_TIM, ENABLE);						//����ARR	
    /*TIMʹ��*/
    TIM_Enable(MCU_RGB_TIM, ENABLE);
    /*test*/
    RGB_SetColorDuty(R, 50);
    RGB_SetColorDuty(G, 50);
    RGB_SetColorDuty(B, 50);
}
void RGB_SetColorDuty(Color color, uint16_t Compare)
{
    switch (color) {
    case R:
        MCU_RGB_RED_TIM_SetCmp(MCU_RGB_TIM, Compare);		//����CCR��ֵ
        break;
    case G:
        MCU_RGB_GREEN_TIM_SetCmp(MCU_RGB_TIM, Compare);
        break;
    case B:
        MCU_RGB_BLUE_TIM_SetCmp(MCU_RGB_TIM, Compare);
        break;

    }

}
