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
 * @file main.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include <rthw.h>
#include <rtthread.h> 
 
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "bsp.h"
#include "usartx.h"

extern void  rt_application_init(void);

/**
 * @brief This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
    rt_hw_board_init();           //初始化硬件板级资源。
    rt_system_scheduler_init(); 	//初始化调度器系统
    rt_system_timer_init();       //初始化系统定时器
#ifdef RT_USING_HEAP
    rt_system_heap_init((void *)N32G45X_SRAM_START, (void *)N32G45X_SRAM_END);    //初始化系统堆内存
#endif //RT_USING_HEAP
    rt_system_timer_thread_init();    //初始化定时器线程
    modbus_task_init();	          // 然后进行对应的参数初始化 
    rt_application_init();        // 只是初始化和启动了线程，但此时调度器尚未启动，因此线程不会立即运行。
    rt_thread_idle_init();        //初始化空闲线程
    rt_system_scheduler_start();  //启动调度器，开始线程调度。
    return ;
}

/**
 * @brief  Main program.
 */
int main(void)
{
    /* disable interrupt first */
    rt_hw_interrupt_disable();

    /* startup RT-Thread RTOS */
    rtthread_startup();
}

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
* @param   expr: If expr is false, it calls assert_failed function which reports
 *         the name of the source file and the source line number of the call
 *         that failed. If expr is true, it returns no value.
 * @param  file: pointer to the source file name.
 * @param  line: assert_param error line source number.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif // USE_FULL_ASSERT
/**
 * @}
 */
