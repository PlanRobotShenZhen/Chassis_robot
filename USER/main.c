#include "system.h"
#include "user_can.h"
#include "motor_data.h"

//�������ȼ�
#define START_TASK_PRIO			4
//�����ջ��С	
#define START_STK_SIZE 			256  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

int main(void)
{ 
   systemInit();
   Motor_Number = 4;
	//������ʼ����
	
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������    
  						
    vTaskStartScheduler();          //�����������
							
}



//��ʼ����������
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //�����ٽ���
    // ���������Ҫ���ڵ��������������
	  xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL);	

	  // �����������
	  xTaskCreate(motor_init_task, "Motor_init_task", MOTOR_STK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);
	  xTaskCreate(motor_task, "Motor_task", MOTOR_STK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);

	  //CANͨ������
	  xTaskCreate(Can_task, "Can_task", CAN_STK_SIZE, NULL, CAN_TASK_PRIO, NULL);	     //RPDO��������Ƶ��1000Hz�������ȼ����


		// ��ȡ�������ϵ�MPU9250������������ݣ��õ����������ǵ�����
	  //xTaskCreate(MPU9250_task, "MPU9250_task", MPU9250_STK_SIZE, NULL, MPU9250_TASK_PRIO, NULL);	
		
		// ��ص�ѹ�ɼ�������Ҫ�������������AD��ѹ�Ļ�ȡ��ʾ 
    //xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL); 
		
		// ����LED�Ŀ���ģ������
    //xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);	     
		
		// ���ڷ������ݽ�������λ��ͨ�ţ��Լ���ģ�������������ݽ�������
    xTaskCreate(data_task, "DATA_task", DATA_STK_SIZE, NULL, DATA_TASK_PRIO, NULL);	 

	// USB���ڷ������ݽ�������λ��ͨ�ţ��Լ���ģ�������������ݽ�������
	xTaskCreate(modbus_task, "ModBUS_task", MODBUS_STK_SIZE, NULL, MODBUS_TASK_PRIO, NULL);	 //����3 ����/���� ��������





    vTaskDelete(StartTask_Handler); //ɾ����ʼ����

    taskEXIT_CRITICAL();            //�˳��ٽ��� 
}


