#ifndef __CircularCar_H
#define __CircularCar_H
#include "stdint.h"
#define Ultrasonic_TIM_Period 				10					//ARR
#define Ultrasonic_TIM_Prescaler 			72						//PSC，10us计数值+1
void Ultrasonic_Start(void);
void Ultrasonic_Init(void);
uint16_t Get_U1distance(void);
uint16_t Get_U2distance(void);
enum IRFunctionCode  // 0-14
{
    Charge_On = 1,
    Charge_Off = 2,
    Charge_Error = 3
};

enum IRSendState
{
    RECEIVED_CHARGEON = 1,
    RECEIVED_CHARGEOFF = 2,
};

enum HighTimeState
{
    WAIT_RISI,
    WAIT_START,
    WAIT_END,
};

// 1.红外对齐
extern EXIO_INPUT exio_input;
extern EXIO_OUTPUT exio_output;

void IR_Init(void);
void IR_SendSignalOn(void);
void IR_SendSignalOff(void);
int IR_ReceiveSignal(void);

// 2.红外通讯
// 红外发射
int get_functioncode(void);
void intToBinary(int functioncode, int* IrSendArray);
void ir_sendzero(void);
void ir_sendone(void);
void ir_sendstart(void);
void ir_sendfunctioncode(int* IrSendArray);
// 红外接收
void get_hightime(void);
void ir_decodefunctioncode(int* IrReceiveArray);
int BinaryToInt(int* IrReceiveArray);
void CarIr_SendDataFcn(uint8_t SendData);
int CarIr_RecvDataFcn(void);


// 电极片
void electrodepad_on(void);
void electrodepad_off(void);
void ElectrodePad_Init(void);

// 主任务
void CircularCar_task(void* pvParameters);
#endif
