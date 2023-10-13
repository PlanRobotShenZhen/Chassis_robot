#include "system.h"
#include "user_can.h"
#include "motor_data.h"

//任务优先级
#define START_TASK_PRIO			4
//任务堆栈大小	
#define START_STK_SIZE 			256  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

int main(void)
{ 
   systemInit();
   Motor_Number = 4;
	//创建开始任务
	
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄    
  						
    vTaskStartScheduler();          //开启任务调度
							
}



//开始任务任务函数
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //进入临界区
    // 这个任务主要用于电机的主流程任务
	  xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL);	

	  // 电机控制任务
	  xTaskCreate(motor_init_task, "Motor_init_task", MOTOR_STK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);
	  xTaskCreate(motor_task, "Motor_task", MOTOR_STK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);

	  //CAN通信任务
	  xTaskCreate(Can_task, "Can_task", CAN_STK_SIZE, NULL, CAN_TASK_PRIO, NULL);	     //RPDO传输任务（频率1000Hz），优先级最高


		// 读取开发板上的MPU9250这个陀螺仪数据，得到六轴陀螺仪的数据
	  //xTaskCreate(MPU9250_task, "MPU9250_task", MPU9250_STK_SIZE, NULL, MPU9250_TASK_PRIO, NULL);	
		
		// 电池电压采集任务，主要是在里面进行了AD电压的获取显示 
    //xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL); 
		
		// 车灯LED的控制模块任务
    //xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);	     
		
		// 串口发送数据进行上下位机通信，以及航模等其他串口数据解析任务
    xTaskCreate(data_task, "DATA_task", DATA_STK_SIZE, NULL, DATA_TASK_PRIO, NULL);	 

	// USB串口发送数据进行上下位机通信，以及航模等其他串口数据解析任务
	xTaskCreate(modbus_task, "ModBUS_task", MODBUS_STK_SIZE, NULL, MODBUS_TASK_PRIO, NULL);	 //串口3 发送/接收 数据任务





    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区 
}


