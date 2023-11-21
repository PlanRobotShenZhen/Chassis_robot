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
#include "motor_data.h"
#include "485_address.h"
#include "balance.h"

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
static struct rt_thread Balance_thread;
static struct rt_thread Motor_init_thread;
static struct rt_thread Motor_thread;
static struct rt_thread Can_thread;
static struct rt_thread DATA_thread;
static struct rt_thread ModBUS_thread;
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
	int i= turn_off_remote;
  rt_err_t result;
  uint16_t* pdu = getPDUData();

	rt_thread_delay(100);   //< 10ms
	LED_Init();                     //初始化与LED连接的硬件接口
	USART1_Init(2000000);	        //=====串口初始化为，普通的串口，打印调试信息 DMA
	Usart3_init(115200);            //上下位机通信初始化，串口3
	Usart5_Init(100000);            //串口5初始化，用于航模控制
	//Adc_Init();                     //采集电池电压ADC引脚初始化	
	Can_Driver_Init();              //底层can协议初始化

	Robot_Select();                 // 根据电位器的值判断目前正在运行的是哪一款机器人，
	// 然后进行对应的参数初始化
	modbus_task_init();
	pdu[car_type] = FourWheel_Car;
	pdu[car_version] = 0x88;
	//初始化航模参数
	rc_ptr = (Remote_Control_struct*)&pdu[i];

	pdu[i++] = TURN_OFF_REMOTE;
	pdu[i++] = TURN_ON_REMOTE;
	pdu[i++] = VEL_BASE_VALUE;
	pdu[i++] = DIR_BASE_VALUE;
	pdu[i++] = LIMIT_MAX_VAL;
	pdu[i++] = LIMIT_MIN_VAL;
	pdu[i++] = SPEED_LEVEL1;
	pdu[i++] = SPEED_LEVEL2;
	pdu[i++] = SPEED_LEVEL3;
	pdu[i++] = SPEED_LOW;
	pdu[i++] = SPEED_MIDDLE;
	pdu[i++] = SPEED_HIGH;
	pdu[i++] = SPEED_DIR_LOW;
	pdu[i++] = SPEED_DIR_MIDDLE;
	pdu[i++] = SPEED_DIR_HIGH;
	pdu[i++] = LIGHT_BASE;
	pdu[i++] = LIGHT_MAX;
	pdu[i++] = LIGHT_MIN;
	//初始化电机参数
	int length = motor2_state - motor1_state;
	motorA_ptr = (Motor_struct*)&pdu[i];
	motorB_ptr = (Motor_struct*)&pdu[i + length];
	motorC_ptr = (Motor_struct*)&pdu[i + 2 * length];
	motorD_ptr = (Motor_struct*)&pdu[i + 3 * length];

	int m_bast_addr;
	uint16_t model = SERVO_ZLAC;//<  测试
	for (int j = 0; j < 4; j++) {
		m_bast_addr = j * length;
		i = motor1_state + m_bast_addr;
		pdu[i++] = 0;					      //节点状态
		pdu[i++] = FourWheer_Radiaus * 10000; //车轮半径，乘以10000后
		pdu[i++] = REDUCTION_RATE * 100;	  //减速比，乘以100后
		pdu[i++] = j + 1;					  //CAN ID
		pdu[i++] = 1000;					  //CAN波特率，除以100后
		pdu[i++] = 0;						  //心跳  
		pdu[i++] = model;					  //伺服厂家型号
		pdu[i++] = 0;						  //目标转矩
		pdu[i++] = 0;                         
		pdu[i++] = 0;						  //目标转速
		pdu[i++] = 0;                         
		pdu[i++] = 0;						  //目标位置
	}
	//初始化小车速度限幅参数
	i = car_max_lin_speed;
	union {
		float v;
		int16_t ud[2];
	}tmp;
	tmp.v = 0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.802;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = 0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	tmp.v = -0.8014;
	pdu[i++] = tmp.ud[1];
	pdu[i++] = tmp.ud[0];
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i++] = 0;
	pdu[i] = 0;
	Motor_Number = 4;

    /* init Balance thread */
    result = rt_thread_init(&Balance_thread, "Balance", Balance_task, RT_NULL, (rt_uint8_t*)&Balance_stack[0], sizeof(Balance_stack), 6, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&Balance_thread);
    }
    /* init Motor_init thread */
    result = rt_thread_init(&Motor_init_thread, "Motor_init", Motor_init_task, RT_NULL, (rt_uint8_t*)&Motor_init_stack[0], sizeof(Motor_init_stack), 10, 10);
    if (result == RT_EOK)
    {
            rt_thread_startup(&Motor_init_thread);
    }
    /* init Motor thread */
    result = rt_thread_init(&Motor_thread, "Motor", Motor_task, RT_NULL, (rt_uint8_t*)&Motor_stack[0], sizeof(Motor_stack), 4, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&Motor_thread);
    }
    /* init Can thread */
    result = rt_thread_init(&Can_thread, "Can", Can_task, RT_NULL, (rt_uint8_t*)&Can_stack[0], sizeof(Can_stack), 3, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&Can_thread);
    }
    /* init DATA thread */
    result = rt_thread_init(&DATA_thread, "DATA", DATA_task, RT_NULL, (rt_uint8_t*)&DATA_stack[0], sizeof(DATA_stack), 8, 5);
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
