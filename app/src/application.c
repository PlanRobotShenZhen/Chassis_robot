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
 * @file application.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <rtthread.h>
#include "led.h"
#include "mb.h"
#include "robot_select_init.h"
#include "usartx.h"
#include "user_can.h"
#include "user_adc.h"
#include "motor_data.h"
#include "485_address.h"
#include "balance.h"
#include "rc_car.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t InitStack[512];
static struct rt_thread InitHandle;

static rt_uint8_t Balance_stack[512];
static rt_uint8_t Motor_init_stack[2048];
static rt_uint8_t Motor_stack[512];
static rt_uint8_t Can_stack[2048];
static rt_uint8_t DATA_stack[512];
static rt_uint8_t ModBUS_stack[512];
static rt_uint8_t ADC_stack[512];
static rt_uint8_t Ultrasonic1_stack[512];
static rt_uint8_t Ultrasonic2_stack[512];
static struct rt_thread Balance_thread;
static struct rt_thread Motor_init_thread;
static struct rt_thread Motor_thread;
static struct rt_thread Can_thread;
static struct rt_thread DATA_thread;
static struct rt_thread ModBUS_thread;
static struct rt_thread ADC_thread;
static struct rt_thread Ultrasonic1_thread;
static struct rt_thread Ultrasonic2_thread;
float Wheel_perimeter = 0;    //轮子周长（单位：米）
float Wheel_spacing = 0;      //主动轮轮距 （单位：米）//后面会在robot_select_init.h文件中初始化
float Axle_spacing = 0;       //前后轴距
float Omni_turn_radiaus = 0;  //全向轮转弯半径

Remote_Control_struct* rc_ptr;
Motor_struct* motorA_ptr;
Motor_struct* motorB_ptr;
Motor_struct* motorC_ptr;
Motor_struct* motorD_ptr;
//Robot_speed *robot_speed_ptr;
enum CarMode g_emCarMode = UNKNOW;                                                //小车类型
unsigned char Flag_Stop = 0;                                                      //1代表使能机器人
enum ENUM_CarControl_Mode g_eControl_Mode = CONTROL_MODE_UNKNOW;  // 机器人的控制方式
unsigned char g_ucRemote_Flag = 0;              //航模开启标志位
unsigned char g_ucRos_Flag = 0;                 // ROS上位机进入标志位 

struct Motor_parameter  MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
// 手动软件复位
void Soft_Reset()
{
	int nDelay = 0;
	__DSB();
	while (nDelay < 100)
	{
		nDelay++;
	}
}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

static void InitTask(void* parameter)
{
    rt_err_t result;
    uint32_t usart1_baud = 2000000;
    uint16_t* pdu = getPDUData();
    //初始化航模参数指针
    rc_ptr = (Remote_Control_struct*)&pdu[turn_off_remote];
    Robot_Select();                 // 根据电位器的值判断目前正在运行的是哪一款机器人，
    //初始化电机参数指针
    motorA_ptr = (Motor_struct*)&pdu[motor1_direction];
    motorB_ptr = (Motor_struct*)&pdu[motor2_direction];
    motorC_ptr = (Motor_struct*)&pdu[motor3_direction];
    motorD_ptr = (Motor_struct*)&pdu[motor4_direction];
    switch (pdu[moddbus_485_baud])
    {
    case 0:usart1_baud = 9600;break;
    case 1:usart1_baud = 19200;break;
    case 2:usart1_baud = 57600;break;
    case 3:usart1_baud = 115200;break;
    case 4:usart1_baud = 256000;break;
    case 5:usart1_baud = 512000;break;
    case 6:usart1_baud = 921600;break;
    case 7:usart1_baud = 1000000;break;
    case 8:usart1_baud = 1500000;break;
    case 9:usart1_baud = 2000000;break;
    }
	rt_thread_delay(100);   //< 10ms
	LED_Init();                     //初始化与LED连接的硬件接口
	USART1_Init(usart1_baud);	        //=====串口初始化为，普通的串口，打印调试信息 DMA
	Usart3_Init(115200);            //上下位机通信初始化，串口3
    Usart4_Init(9600);              //串口4初始化，用于读取电池信息
    Usart5_Init(100000);            //串口5初始化，用于航模控制
    if (pdu[car_model] == RC_Car)Adc_Init();                     //采集电池电压ADC引脚初始化	
	Can_Driver_Init(pdu[CAN_baud]);              //底层can协议初始化
    if(pdu[car_model]== RC_Car)RCCAR_Init(pdu);
	

    /* init Balance thread */
    result = rt_thread_init(&Balance_thread, "Balance", Balance_task, (void*)pdu, (rt_uint8_t*)&Balance_stack[0], sizeof(Balance_stack), 6, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&Balance_thread);
    }
    if (pdu[car_model] != RC_Car)
    {

        /* init Motor_init thread */
        result = rt_thread_init(&Motor_init_thread, "Motor_init", Motor_init_task, (void*)pdu, (rt_uint8_t*)&Motor_init_stack[0], sizeof(Motor_init_stack), 10, 10);
        if (result == RT_EOK)
        {
            rt_thread_startup(&Motor_init_thread);
        }
        /* init Motor thread */
        result = rt_thread_init(&Motor_thread, "Motor", Motor_task, (void*)pdu, (rt_uint8_t*)&Motor_stack[0], sizeof(Motor_stack), 4, 5);
        if (result == RT_EOK)
        {
            rt_thread_startup(&Motor_thread);
        }
        /* init Can thread */
        result = rt_thread_init(&Can_thread, "Can", Can_task, (void*)pdu, (rt_uint8_t*)&Can_stack[0], sizeof(Can_stack), 3, 5);
        if (result == RT_EOK)
        {
            rt_thread_startup(&Can_thread);
        }
        /* init adc thread */
        result = rt_thread_init(&ADC_thread, "ADC", ADC_task, (void*)pdu, (rt_uint8_t*)&ADC_stack[0], sizeof(ADC_stack), 11, 12);
        if (result == RT_EOK)
        {
            rt_thread_startup(&ADC_thread);
        }
    }
    /* init DATA thread */
    result = rt_thread_init(&DATA_thread, "DATA", DATA_task, (void*)pdu, (rt_uint8_t*)&DATA_stack[0], sizeof(DATA_stack), 8, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&DATA_thread);
    }
    /* init ModBUS thread */
    result = rt_thread_init(&ModBUS_thread, "ModBUS", ModBUS_task, RT_NULL, (rt_uint8_t*)&ModBUS_stack[0], sizeof(ModBUS_stack), 7, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&ModBUS_thread);
    }  

    if (pdu[car_model] == Diff_Car)
    {
        result = rt_thread_init(&Ultrasonic1_thread, "Ultrasonic", Ultrasonic1_task, (void*)pdu, (rt_uint8_t*)&Ultrasonic1_stack[0], sizeof(Ultrasonic1_stack), 13, 14);
        if (result == RT_EOK)
        {
            rt_thread_startup(&Ultrasonic1_thread);
        }
        result = rt_thread_init(&Ultrasonic2_thread, "Ultrasonic", Ultrasonic2_task, (void*)pdu, (rt_uint8_t*)&Ultrasonic2_stack[0], sizeof(Ultrasonic2_stack), 13, 14);
        if (result == RT_EOK)
        {
            rt_thread_startup(&Ultrasonic2_thread);
        }
    }

    
}

/**
 * @brief  init application
 */
void rt_application_init(void)
{
    rt_err_t result;

    result = rt_thread_init(&InitHandle, "Init", InitTask, RT_NULL, (rt_uint8_t*)&InitStack[0], sizeof(InitStack), 2, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&InitHandle);
    }
   
}

/*@}*/
