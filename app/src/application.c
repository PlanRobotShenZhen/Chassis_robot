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
#include "Charger.h"
#include "remote.h"
#include "motor.h"
#include "RJ_JT.h"
#include "CircularCar.h"
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

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t InitStack[512];
static struct rt_thread InitHandle;

static rt_uint8_t Balance_stack[512];
static rt_uint8_t Motor_stack[2048];
static rt_uint8_t Remote_stack[512];
static rt_uint8_t CAN_stack[2048];
static rt_uint8_t Modbus_stack[512];
static rt_uint8_t ADC_stack[512];
static rt_uint8_t LED_stack[512];
static rt_uint8_t RJJT_stack[512];
static rt_uint8_t Charge_stack[512];
static rt_uint8_t CircularCar_stack[512];
static struct rt_thread Balance_thread;
static struct rt_thread Motor_thread;
static struct rt_thread Remote_thread;
static struct rt_thread CAN_thread;
static struct rt_thread LED_thread;
static struct rt_thread RJJT_thread;
static struct rt_thread Modbus_thread;
static struct rt_thread ADC_thread;
static struct rt_thread Charge_thread;
static struct rt_thread CircularCar_thread;



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

uint32_t GetUsart1_baud(uint16_t data)
{
    switch (data)
    {
        case 0:
            return 9600;

        case 1:
            return 19200;

        case 2:
            return 57600;

        case 3:
            return 115200;

        case 4:
            return 256000;

        case 5:
            return 512000;

        case 6:
            return 921600;

        case 7:
            return 1000000;

        case 8:
            return 1500000;

        case 9:
            return 2000000;
    }

    return 2000000;
}

void DiyFunctionBasedCar(uint16_t data)
{
    switch (data)
    {
        case Akm_Car:
            Adc_Init();                     //采集电池电压ADC引脚初始化
            break;

        case Diff_Car: //圆形底盘
            Usart2_Init(9600);              //串口2初始化，圆形底盘用串口2代替串口4，用于读取电池信息
            Ultrasonic_Init();
            Adc_Init();
            IR_Init();
            ElectrodePad_Init();
            //GPIO_SetBits(MCU_CHARGE_ON_GPIO, MCU_CHARGE_ON_PIN);    //圆形底盘充电电极片,对接成功后才开启，默认关闭
            pdu[ros_chargingcommand] = 1;//圆形底盘没有电池SOC传感器，默认开启充电信号，红外一直工作。
            break;

        case FourWheel_Car:
            Adc_Init();
            break;

        case RC_Car:
            RCCAR_Init();
            break;

        case Charger:
            ChargerBalanceInit();
            NVIC_Config();
            IR_RX_Init();
            IC_Init();
            JTAG_Set(SWD_ENABLE);
            break;

        default:
            break;
    }
}

static void InitTask(void* parameter)
{
    rt_err_t result;
    systemInit();

    /* init Balance thread */

    result = rt_thread_init(&Balance_thread, "Balance", Balance_task, (void*)pdu, (rt_uint8_t*)&Balance_stack[0], sizeof(Balance_stack), 6, 5);

    if (result == RT_EOK)   rt_thread_startup(&Balance_thread);

    if ((pdu[car_type] != Charger) && (pdu[car_type] != RC_Car))
    {
        /* init Remote thread */
        result = rt_thread_init(&Remote_thread, "Remote", Remote_Task, (void*)pdu, (rt_uint8_t*)&Remote_stack[0], sizeof(Remote_stack), 5, 5);

        if (result == RT_EOK)
            rt_thread_startup(&Remote_thread);

        /* init Modbus thread */
        result = rt_thread_init(&Modbus_thread, "Modbus", Modbus_task, RT_NULL, (rt_ uint8_t*)&Modbus_stack[0], sizeof(Modbus_stack), 7, 5);

        if (result == RT_EOK)
            rt_thread_startup(&Modbus_thread);

        /* init Motor thread */
        result = rt_thread_init(&Motor_thread, "Motor", Motor_task, (void*)pdu, (rt_uint8_t*)&Motor_stack[0], sizeof(Motor_stack), 4, 5);

        if (result == RT_EOK)
            rt_thread_startup(&Motor_thread);

        /* init Can thread */
        result = rt_thread_init(&CAN_thread, "Can", Can_task, (void*)pdu, (rt_uint8_t*)&CAN_stack[0], sizeof(CAN_stack), 3, 5);

        if (result == RT_EOK)
            rt_thread_startup(&CAN_thread);

        /* init adc thread */
        result = rt_thread_init(&ADC_thread, "ADC", ADC_task, (void*)pdu, (rt_uint8_t*)&ADC_stack[0], sizeof(ADC_stack), 11, 12);

        if (result == RT_EOK)
            rt_thread_startup(&ADC_thread);

        // init RJJT thread      //急停开关
        result = rt_thread_init(&RJJT_thread, "RJJT_task", RJJT_task, (void*)pdu, (rt_uint8_t*)&RJJT_stack[0], sizeof(RJJT_stack), 2, 5);

        if (result == RT_EOK)
            rt_thread_startup(&RJJT_thread);

        // init Led thread
        result = rt_thread_init(&LED_thread, "Led_task", Led_task, (void*)pdu, (rt_uint8_t*)&LED_stack[0], sizeof(LED_stack), 9, 5);

        if (result == RT_EOK)
            rt_thread_startup(&LED_thread);
    }

    if (pdu[car_type] == Diff_Car) //  圆形差速度底盘   与自动 充电相关，所以要增加这样一个函数
    {
        // init CircularCar thread
        result = rt_thread_init(&CircularCar_thread, "CircularCar_task", CircularCar_task, (void*)pdu, (rt_uint8_t*)&CircularCar_stack[0], sizeof(CircularCar_stack), 8, 5);

        if (result == RT_EOK)
            rt_thread_startup(&CircularCar_thread);
    }

    if (pdu[car_type] == Charger)//充电桩与圆盘 两选一
    {
        // init ChargeStation thread
        result = rt_thread_init(&Charge_thread, "Charge_task", Charge_task, (void*)pdu, (rt_uint8_t*)&Charge_stack[0], sizeof(Charge_stack), 10, 5);

        if (result == RT_EOK)
            rt_thread_startup(&Charge_thread);
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
