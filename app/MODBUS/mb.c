/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mb.c,v 1.28 2010/06/06 13:54:40 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "usartx.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbfunc.h"

#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif
#include "motor_data.h"
/* ----------------------- Static variables ---------------------------------*/

static UCHAR    ucMBAddress;
/* 私有变量 ------------------------------------------------------------------*/
PDUData_TypeDef PduData;
REG_VALUE R_value;
static uint16_t mbdata[MB_RTU_DATA_MAX_SIZE];
extern uint16_t error_code;
static enum
{
    STATE_ENABLED,
    STATE_DISABLED,
    STATE_NOT_INITIALIZED
} eMBState = STATE_NOT_INITIALIZED;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */
static peMBFrameSend peMBFrameSendCur;
static pvMBFrameStart pvMBFrameStartCur;
static pvMBFrameStop pvMBFrameStopCur;
static peMBFrameReceive peMBFrameReceiveCur;
static pvMBFrameClose pvMBFrameCloseCur;

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */
BOOL( *pxMBFrameCBByteReceived ) ( void );
BOOL( *pxMBFrameCBTransmitterEmpty ) ( void );
BOOL( *pxMBPortCBTimerExpired ) ( void );

BOOL( *pxMBFrameCBReceiveFSMCur ) ( void );
BOOL( *pxMBFrameCBTransmitFSMCur ) ( void );

eMBErrorCode eMBEnable(void);
/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};
/**
  * 函数功能: 填充内存
  * 输入参数: buf:内存空间首地址,Code:功能码
  * 返 回 值: 无
  * 说    明: 功能不同的功能码填充内容不一的内存空间,
  */
void FillBuf(uint16_t* buf, uint8_t Code)
{
    uint16_t i = 0;
    switch (Code)
    {
    case FUN_CODE_03H:
    case FUN_CODE_06H:
    case FUN_CODE_10H:
    case FUN_CODE_11H:
        for (i = 0; i < MB_RTU_DATA_MAX_SIZE; i++)
            buf[i] = 0;
        break;
    }
}
/**
  * 函数功能: 判断操作的数据量是否符合协议范围
  * 输入参数: _RegNum:寄存器数量,_FunCode:功能码,_ByteNum:字节数量
  * 返 回 值: 异常码:03或NONE
  * 说    明: 对可操作连续内存空间的功能码需要验证操作的地址是否符合范围
  */
uint8_t MB_JudgeNum(uint16_t _RegNum, uint8_t _FunCode, uint16_t _ByteNum)
{
    uint8_t Excode = EX_CODE_NONE;
    uint16_t _CoilNum = _RegNum; // 线圈(离散量)的数量
    switch (_FunCode)
    {
    case FUN_CODE_01H:
    case FUN_CODE_02H:
        if ((_CoilNum < 0x0001) || (_CoilNum > 0x07D0))
            Excode = EX_CODE_03H;// 异常码03H;
        break;
    case FUN_CODE_03H:
    case FUN_CODE_04H:
        if ((_RegNum < 0x0001) || (_RegNum > 0x007D))
            Excode = EX_CODE_03H;// 异常码03H;      
        break;
    case FUN_CODE_10H:
        if ((_RegNum < 0x0001) || (_RegNum > 0x007B))
            Excode = EX_CODE_03H;// 异常码03H
        if (_ByteNum != (_RegNum << 1))
            Excode = EX_CODE_03H;// 异常码03H
        break;
    case FUN_CODE_11H:
        if ((_RegNum < 0x0001) || (_RegNum > 0x04CE))
            Excode = EX_CODE_03H;// 异常码03H
        //if (_ByteNum != (_RegNum << 1))
        //    Excode = EX_CODE_03H;// 异常码03H
        break;
    }
    return Excode;
}

/**
  * 函数功能: 判断地址是否符合协议范围
  * 输入参数: _Addr:起始地址,_RegNum:寄存器数量,_FunCode:功能码
  * 返 回 值: 异常码:02H或NONE
  * 说    明: 地址范围是0x0000~0xFFFF,可操作的空间范围不能超过这个区域
  */
uint8_t MB_JudgeAddr(uint16_t _Addr, uint16_t _RegNum)
{
    uint8_t Excode = EX_CODE_NONE;
    /* 地址+寄存器数量不能超过0xFFFF */
    if (((uint32_t)_RegNum + (uint32_t)_Addr) > (uint32_t)0xFFFF)
    {
        Excode = EX_CODE_02H;// 异常码 02H
    }
    return Excode;
}

/**
  * 函数功能: 对接收到的数据进行分析并执行
  * 输入参数: 无
  * 返 回 值: 异常码或0x00
  * 说    明: 判断功能码,验证地址是否正确.数值内容是否溢出,数据没错误就发送响应信号
  */
uint8_t MB_Analyze_Execute(void)
{
    uint16_t ExCode = EX_CODE_NONE;
    /* 校验功能码 */
    if (IS_NOT_FUNCODE(PduData.Code)) // 不支持的功能码
    {
        /* Modbus异常响应 */
        ExCode = EX_CODE_01H;            // 异常码01H
        return ExCode;
    }
    /* 根据功能码分别做判断 */
    switch (PduData.Code)
    {
        /* 这里认为01H功能码和02功能码是一样的,其实也没什么不一样
         * 只是操作地址可能不一样,这一点结合具体来实现,可以在main函数
         * 申请单独的内存使用不同的功能码,在实际应用中必须加以区分使用
         * 不同的内存空间
         */
         /* ---- 01H  02H 读取离散量输入(Coil Input)---------------------- */
    case FUN_CODE_01H:
    case FUN_CODE_02H:
        /* 判断线圈数量是否正确 */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, 1);
        if (ExCode != EX_CODE_NONE)
            return ExCode;

        /* 判断地址是否正确*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
        /* ---- 03H  04H 读取保持/输入寄存器---------------------- */
    case FUN_CODE_03H:
    case FUN_CODE_04H:
        /* 判断寄存器数量是否正确 */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, PduData.byteNums);
        if (ExCode != EX_CODE_NONE)
            return ExCode;

        /* 判断地址是否正确*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
        /* ---- 05H 写入单个离散量---------------------- */
    case FUN_CODE_05H:
        break;
        /* ---- 06H 写单个保持寄存器 ---------------------- */
    case FUN_CODE_06H:
        break;
        /* ---- 10H 写多个保持寄存器 ---------------------- */
    case FUN_CODE_10H:
        /* 判断寄存器数量是否正确 */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, PduData.byteNums);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        /* 判断地址是否正确*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
        /* ---- 11H 写多个保持寄存器 ---------------------- */
    case FUN_CODE_11H:
        /* 判断寄存器数量是否正确 */
        ExCode = MB_JudgeNum(PduData.Num, PduData.Code, PduData.byteNums);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        /* 判断地址是否正确*/
        ExCode = MB_JudgeAddr(PduData.Addr, PduData.Num);
        if (ExCode != EX_CODE_NONE)
            return ExCode;
        break;
    }
    /* 数据帧没有异常 */
    return ExCode; //   EX_CODE_NONE
}


static uint16_t MB_RSP_AAH(uint16_t _TxCount, uint16_t* _AddrOffset, uint16_t _RegNum)
{
    /* 返回保持寄存器内的数据 */
    for (uint8_t i = 0; i < _RegNum; i++)
    {
        uart1_send_data[_TxCount++] = ((*_AddrOffset) >> 8);
        uart1_send_data[_TxCount++] = *_AddrOffset++;
    }

    return _TxCount;

}
/**
  * 函数功能: 读取线圈状态（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_CoilNum:线圈数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取离散输出,并且填充Tx_Buf
  */
static uint16_t MB_RSP_01H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _CoilNum)
{
    /*
        主机发送:
            01 从机地址
            01 功能码
            00 寄存器起始地址高字节
            02 寄存器起始地址低字节
            00 寄存器数量高字节
            08 寄存器数量低字节
            9C CRC校验高字节
            0C CRC校验低字节

        从机应答: 	1代表ON，0代表OFF（使用LED的状态来代替）。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
            01 从机地址
            01 功能码
            01 返回字节数
            02 数据1(线圈0002H-线圈0011H)
            D0 CRC校验高字节
            49 CRC校验低字节

        例子1:
        发送：	01 01 00 02 00 08   9C 0C	  --- 查询D02开始的8个继电器状态
        返回：	01 01 01 01         90 48   --- 查询到8个状态为：0000 0001 第二个LED为亮

        例子2:
        发送：	01 01 00 01 00 10   6C 06	  --- 查询D01开始的16个继电器状态
        返回：	01 01 02 FF FF      B8 4C   --- 查询到两个字节数据为0xFFFF

    */
    //  uint16_t i = 0;
    //	uint16_t m;	
    //	uint8_t status[10];	
    //	
    //  /* 计算返回字节数（_CoilNum变量是以位为单位） */
    //  m = (_CoilNum+7)/8;
    //  /* 返回字节数（数量）*/
    //	Tx_Buf[_TxCount++] = m; 
    //	if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
    //  {
    //		/* 将获取的线圈状态首先清零 */
    //		for (i = 0; i < m; i++)
    //		{
    //			status[i] = 0;
    //		}		
    //		/* 获取对应线圈状态，并将其写入status[] */
    //		for (i = 0; i < _CoilNum; i++)
    //		{
    //			/* 读LED的状态，写入状态寄存器的每一位 */
    //			if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))		
    //			{  
    //				status[i / 8] |= (1 << (i % 8));
    //			}			
    //		}    		
    //	}
    //	/* 填充发送内容 */
    //	for (i = 0; i < m; i++)
    //	{
    //		Tx_Buf[_TxCount++] = status[i];	/* 继电器状态 */
    //	}		

        /*----------------------------分割线----------------------------------*/
    uart1_send_data[_TxCount++] = 2;
    /* 填充返回内容 */
    if (_AddrOffset == COIL_D01)
    {
        uart1_send_data[_TxCount++] = R_value.D01 >> 8;
        uart1_send_data[_TxCount++] = R_value.D01;
    }
    else if (_AddrOffset == COIL_D02)
    {
        uart1_send_data[_TxCount++] = R_value.D02 >> 8;
        uart1_send_data[_TxCount++] = R_value.D02;
    }
    else if (_AddrOffset == COIL_D03)
    {
        uart1_send_data[_TxCount++] = R_value.D03 >> 8;
        uart1_send_data[_TxCount++] = R_value.D03;
    }
    else if (_AddrOffset == COIL_D04)
    {
        uart1_send_data[_TxCount++] = R_value.D04 >> 8;
        uart1_send_data[_TxCount++] = R_value.D04;
    }
    else
    {
        uart1_send_data[_TxCount++] = 0;
        uart1_send_data[_TxCount++] = 0;
    }

    return _TxCount;
}

/**
  * 函数功能: 读取离散输入（只读）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_CoilNum:线圈数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取离散输出,并且填充Tx_Buf
  */
static uint16_t MB_RSP_02H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _CoilNum)
{
    /*
        主机发送:
            01 从机地址
            02 功能码
            00 寄存器起始地址高字节
            01 寄存器起始地址低字节
            00 寄存器数量高字节
            08 寄存器数量低字节
            28 CRC校验高字节
            0C CRC校验低字节

        从机应答: 	1代表ON，0代表OFF（使用LED的状态来代替）。若返回的线圈数不为8的倍数，则在最后数据字节未尾使用0代替. BIT0对应第1个
            01 从机地址
            02 功能码
            01 返回字节数
            02 数据1(线圈0002H-线圈0011H)
            D0 CRC校验高字节
            49 CRC校验低字节

        例子:
        发送：	01 01 00 02 00 08   9C 0C	  --- 查询D02开始的8个继电器状态
        返回：	01 01 01 02         D0 49   --- 查询到8个状态为：0000 0010 第二个LED为亮
    */
    uint16_t i = 0;
    uint16_t m;
    uint8_t status[10];

    /* 计算返回字节数（_CoilNum变量是以位为单位） */
    m = (_CoilNum + 7) / 8;
    /* 返回字节数（数量）*/
    uart1_send_data[_TxCount++] = m;

    if ((_AddrOffset >= COIL_D01) && (_CoilNum > 0))
    {
        /* 将获取的线圈状态首先清零 */
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }
        /* 获取对应线圈状态，并将其写入status[] */
        //for (i = 0; i < _CoilNum; i++)
        //{
        //    /* 读LED的状态，写入状态寄存器的每一位 */
        //    if (Get_LEDx_State(i + 1 + _AddrOffset - COIL_D01))
        //    {
        //        status[i / 8] |= (1 << (i % 8));
        //    }
        //}
    }
    /* 填充发送内容 */
    for (i = 0; i < m; i++)
    {
        /* 继电器状态 */
        uart1_send_data[_TxCount++] = status[i];
    }
    return _TxCount;
}

/**
  * 函数功能: 读取保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取保持寄存器的内容,并且填充Tx_Buf
  */
static uint8_t MB_RSP_03H(uint16_t _TxCount, uint16_t* _AddrOffset, uint16_t _RegNum)
{
    /*
        从机地址为01H。保持寄存器的起始地址为0010H，结束地址为0011H。该次查询总共访问2个保持寄存器。
        主机发送:
            01 从机地址
            03 功能码
            00 寄存器地址高字节
            10 寄存器地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            C5 CRC高字节
            CE CRC低字节

        从机应答: 	保持寄存器的长度为2个字节。对于单个保持寄存器而言，寄存器高字节数据先被传输，
                    低字节数据后被传输。保持寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
            01 从机地址
            03 功能码
            04 字节数
            12 数据1高字节(0010H)
            34 数据1低字节(0010H)
            02 数据2高字节(0011H)
            03 数据2低字节(0100H)
            FF CRC高字节
            F4 CRC低字节

        读一个保持寄存器例子:
            发送：	01 03 00 10 00 01            85 CF ---- 读 0010H一个寄存器内容
            返回：	01 03 02 12 34               B5 33 ---- 返回10H功能码写入的内容（10H功能码会介绍）
    */

    /* 填充返回寄存器数量 */
    uart1_send_data[_TxCount++] = _RegNum * 2;
    /* 返回保持寄存器内的数据 */
    for (uint8_t i = 0; i < _RegNum; i++)
    {
        uart1_send_data[_TxCount++] = ((*_AddrOffset) >> 8);
        uart1_send_data[_TxCount++] = *_AddrOffset++;
    }

    return _TxCount;
}

/**
  * 函数功能: 读取输入寄存器（与上述03H指令类似，返回内容也类似）（只读）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 读取保持寄存器的内容,并且填充Tx_Buf
  */
static uint8_t MB_RSP_04H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegNum)
{
    /*
        主机发送:
            01 从机地址
            04 功能码
            00 寄存器起始地址高字节
            20 寄存器起始地址低字节
            00 寄存器个数高字节
            02 寄存器个数低字节
            70 CRC高字节
            01 CRC低字节

        从机应答:  输入寄存器长度为2个字节。对于单个输入寄存器而言，寄存器高字节数据先被传输，
                低字节数据后被传输。输入寄存器之间，低地址寄存器先被传输，高地址寄存器后被传输。
            01 从机地址
            04 功能码
            02 字节数
            02 数据1高字节(0020H)
            03 数据1低字节(0020H)
            00 数据2高字节(0021H)
            00 数据2低字节(0021H)
            82 CRC高字节
            3C CRC低字节

        例子:
            发送：	01 04 00 20 00 02      70 01  --- 读 0020H IN1 开始的2个字节数据内容
            返回：	01 04 02 02 03 00 00   82 3C  --- 返回：02 03 00 00 4个数据（按下KEY1按键后改变 R_value.IN1的值）
    */
    uint8_t i;
    uint16_t reg_value[64];
    /* 填充返回寄存器数量 */
    uart1_send_data[_TxCount++] = _RegNum;
    /* 读取保持寄存器内容 */
    for (i = 0; i < _RegNum; i++)
    {
        switch (_AddrOffset)
        {
            /* 测试参数 */
        case REG_IN1:
            reg_value[i] = R_value.IN1;
            break;

        default:
            reg_value[i] = 0;
            break;
        }
        _AddrOffset++;
    }

    /* 填充返回内容 */
    for (i = 0; i < _RegNum; i++)
    {

        uart1_send_data[_TxCount++] = reg_value[i] >> 8;
        uart1_send_data[_TxCount++] = reg_value[i] & 0xFF;
    }
    return _TxCount;
}

/**
  * 函数功能: 写单个线圈（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegDATA:写入数据
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_05H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegDATA)
{
    /*
        主机发送: 写单个线圈寄存器。简单的01~03寄存器地址对应LED1~LED3,将数据存放于D01、D02、D03、D04等4个成员中
        05H指令设置单个线圈的状态
            01 从机地址
            05 功能码
            00 寄存器地址高字节
            01 寄存器地址低字节
            FF 数据1高字节
            FF 数据1低字节
            9D CRC校验高字节
            BA CRC校验低字节

        从机应答:
            01 从机地址
            05 功能码
            00 寄存器地址高字节
            01 寄存器地址低字节
            FF 寄存器1高字节
            FF 寄存器1低字节
            9D CRC校验高字节
            BA CRC校验低字节

        例子:
        发送：	01 05 00 04 FF FF   8D BB   -- 发送数据0xFFFF至0x04地址线圈中
        返回：	01 05 00 04 FF FF   8D BB   -- 返回原始数据
    */

    /* 填充地址值 */
    uart1_send_data[_TxCount++] = _AddrOffset >> 8;
    uart1_send_data[_TxCount++] = _AddrOffset;

    if (_AddrOffset == COIL_D01)
    {
        R_value.D01 = _RegDATA;
        //LED1_ON;
    }
    else if (_AddrOffset == COIL_D02)
    {
        R_value.D02 = _RegDATA;
        //LED2_ON;
    }
    else if (_AddrOffset == COIL_D03)
    {
        R_value.D03 = _RegDATA;
        //LED3_ON;
    }
    else if (_AddrOffset == COIL_D04)
    {
        R_value.D04 = _RegDATA;
        //LED1_TOGGLE;
        //LED2_TOGGLE;
        //LED3_TOGGLE;
    }
    else
    {
        R_value.D01 = 0;
        R_value.D02 = 0;
        R_value.D03 = 0;
        R_value.D04 = 0;
    }

    uart1_send_data[_TxCount++] = _RegDATA >> 8;
    uart1_send_data[_TxCount++] = _RegDATA;
    return _TxCount;
}

int isWritableAddr(uint16_t _AddrOffset)
{
    if (_AddrOffset >= 150&& _AddrOffset<=250)
    {
        return 0;
    }
    return 1;
}

/**
  * 函数功能: 写单个保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset:地址偏移量,_RegNum: 写入数据，_AddrAbs：保持寄存器地址
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_06H(uint16_t _TxCount, uint16_t _AddrOffset, uint16_t _RegNum, uint16_t* _AddrAbs)
{
    /*
        写保持寄存器。注意06指令只能操作单个保持寄存器，10H指令可以设置单个或多个保持寄存器
        主机发送:
            01 从机地址
            06 功能码
            00 寄存器地址高字节
            10 寄存器地址低字节
            67 数据1高字节
            4A 数据1低字节
            23 CRC校验高字节
            C8 CRC校验低字节

        从机响应:
            01 从机地址
            06 功能码
            00 寄存器地址高字节
            10 寄存器地址低字节
            67 数据1高字节
            4A 数据1低字节
            23 CRC校验高字节
            C8 CRC校验低字节

        例子:
            发送：	01 06 00 10 67 4A  23 C8    ---- 将0010地址寄存器设置为67 4A
            返回：	01 06 00 10 67 4A  23 C8    ---- 返回同样数据
*/
/* 填充地址值 */
    uart1_send_data[_TxCount++] = _AddrOffset >> 8;
    uart1_send_data[_TxCount++] = _AddrOffset;

    /* 将数据写入保持寄存器内 */
    if (isWritableAddr(_AddrOffset))
    {//<  可写地址
        *_AddrAbs = _RegNum;
    }
    if (_AddrOffset == 6 && _RegNum == 0xa5)
    {
        Soft_Reset();
    }
    else if (_AddrOffset == 3 && _RegNum == 0)
    {
        error_code = 0;
    }
    /* 填充返回内容 */
    uart1_send_data[_TxCount++] = PduData.Num >> 8;
    uart1_send_data[_TxCount++] = PduData.Num;

    return _TxCount;
}

/**
  * 函数功能: 写多个保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量，_Datebuf:数据
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_10H(uint16_t _TxCount, uint16_t  _AddrOffset, uint16_t _RegNum, uint16_t* _AddrAbs, uint8_t* _Datebuf)
{
    /*
        主机发送:
            01 从机地址
            10 功能码
            00 寄存器起始地址高字节
            10 寄存器起始地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            04 字节数
            12 数据1高字节
            34 数据1低字节
            02 数据2高字节
            03 数据2低字节
            F7 CRC校验高字节
            74 CRC校验低字节

        从机响应:
            01 从机地址
            10 功能码
            00 寄存器地址高字节
            10 寄存器地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            40 CRC校验高字节
            0D CRC校验低字节

        例子:
            发送：	01 10 00 10 00 02 04 12 34 02 03 F7 74    ----  向0010H~0011H写入12 34 02 03 四个字节数据
            返回：	01 10 00 10 00 02 40 0D                   ----  返回内容

    */
    uint16_t i = 0;
    uint16_t Value = 0;
    /* 填充地址值 */
    uart1_send_data[_TxCount++] = _AddrOffset >> 8;
    uart1_send_data[_TxCount++] = _AddrOffset;

    /* 写入多个保持寄存器 */
    for (i = 0; i < _RegNum; i++)
    {
        Value = (uint16_t)((*_Datebuf << 8) | (*(_Datebuf + 1)));
        *_AddrAbs++ = Value;
        _Datebuf += 2;
    }

    uart1_send_data[_TxCount++] = _RegNum >> 8;
    uart1_send_data[_TxCount++] = _RegNum;
    return _TxCount;
}

/**
  * 函数功能: 写多个保持寄存器（读/写）
  * 输入参数: _TxCount :发送计数器,_AddrOffset地址偏移量,_RegNum:寄存器数量，_Datebuf:数据
  * 返 回 值: Tx_Buf的数组元素坐标
  * 说    明: 填充Tx_Buf
  */
static uint8_t MB_RSP_11H(uint16_t _TxCount, uint16_t  _AddrOffset, uint16_t _RegNum, uint16_t* _AddrAbs, uint8_t* _Datebuf)
{
    /*
        主机发送:
            01 从机地址
            10 功能码
            00 寄存器起始地址高字节
            10 寄存器起始地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            04 字节数 //《 实际不使用，无效
            12 数据1高字节
            34 数据1低字节
            02 数据2高字节
            03 数据2低字节
            F7 CRC校验高字节
            74 CRC校验低字节

        从机响应:
            01 从机地址
            10 功能码
            00 寄存器地址高字节
            10 寄存器地址低字节
            00 寄存器数量高字节
            02 寄存器数量低字节
            40 CRC校验高字节
            0D CRC校验低字节

        例子:
            发送：	01 10 00 10 00 02 04 12 34 02 03 F7 74    ----  向0010H~0011H写入12 34 02 03 四个字节数据
            返回：	01 10 00 10 00 02 40 0D                   ----  返回内容

    */
    uint16_t i = 0;
    uint16_t Value = 0;
    /* 填充地址值 */
    uart1_send_data[_TxCount++] = _AddrOffset >> 8;
    uart1_send_data[_TxCount++] = _AddrOffset;

    /* 写入多个保持寄存器 */
    for (i = 0; i < _RegNum; i++)
    {
        Value = (uint16_t)((*_Datebuf << 8) | (*(_Datebuf + 1)));
        *_AddrAbs++ = Value;
        _Datebuf += 2;
    }

    uart1_send_data[_TxCount++] = _RegNum >> 8;
    uart1_send_data[_TxCount++] = _RegNum;
    return _TxCount;
}

/**
  * 函数功能: 异常响应
  * 输入参数: _FunCode :发送异常的功能码,_ExCode:异常码
  * 返 回 值: 无
  * 说    明: 当通信数据帧发生异常时,发送异常响应
  */
void MB_Exception_RSP(uint8_t _FunCode, uint8_t _ExCode)
{
    uint16_t crc;
    uart1_send_len = 0;
    uart1_send_data[uart1_send_len++] = ucMBAddress;		    /* 从站地址 */
    uart1_send_data[uart1_send_len++] = _FunCode | 0x80;		  /* 功能码 + 0x80*/
    uart1_send_data[uart1_send_len++] = _ExCode;	          /* 异常码*/

    crc = usMBCRC16((UCHAR*)&uart1_send_data, uart1_send_len);
    uart1_send_data[uart1_send_len++] = crc;	          /* crc 低字节 */
    uart1_send_data[uart1_send_len++] = crc >> 8;		      /* crc 高字节 */
    //UART_Tx((uint8_t*)Tx_Buf, TxCount);
    uart1_send_flag = 2;
}
/**
  * 函数功能: 正常响应
  * 输入参数: _FunCode :功能码
  * 返 回 值: 无
  * 说    明: 当通信数据帧没有异常时并且成功执行之后,发送响应数据帧
  */
void MB_RSP(uint8_t _FunCode)
{
    uart1_send_len = 0;
    uint16_t crc = 0;	
    uart1_send_data[uart1_send_len++] = ucMBAddress;		 /* 从站地址 */
    uart1_send_data[uart1_send_len++] = _FunCode;        /* 功能码   */
    switch (_FunCode)
    {
    case FUN_CODE_AAH:
        PduData.Addr = 218;
        PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // 保持寄存器的起始地址
        uart1_send_len = MB_RSP_AAH(uart1_send_len, (uint16_t*)PduData.PtrHoldingOffset,7);
        break;
    case FUN_CODE_01H:
        /* 读取线圈状态 */
        uart1_send_len = MB_RSP_01H(uart1_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_02H:
        /* 读取离散输入 */
        uart1_send_len = MB_RSP_02H(uart1_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_03H:
        /* 读取保持寄存器 */
        uart1_send_len = MB_RSP_03H(uart1_send_len, (uint16_t*)PduData.PtrHoldingOffset, PduData.Num);
        break;
    case FUN_CODE_04H:
        /* 读取输入寄存器 */
        uart1_send_len = MB_RSP_04H(uart1_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_05H:
        /* 写单个线圈 */
        uart1_send_len = MB_RSP_05H(uart1_send_len, PduData.Addr, PduData.Num);
        break;
    case FUN_CODE_06H:
        /* 写单个保持寄存器 */
        uart1_send_len = MB_RSP_06H(uart1_send_len, PduData.Addr, PduData.Num, (uint16_t*)PduData.PtrHoldingOffset);
        break;
    case FUN_CODE_10H:
        /* 写多个保持寄存器 */
        uart1_send_len = MB_RSP_10H(uart1_send_len, PduData.Addr, PduData.Num, (uint16_t*)PduData.PtrHoldingOffset, (uint8_t*)PduData.ValueReg);
        break;
    case FUN_CODE_11H:
        /* 写多个保持寄存器 */
        uart1_send_len = MB_RSP_11H(uart1_send_len, PduData.Addr, PduData.Num, (uint16_t*)PduData.PtrHoldingOffset, (uint8_t*)PduData.ValueReg);
        break;
    }
    crc = usMBCRC16((UCHAR*)&uart1_send_data, uart1_send_len);
    uart1_send_data[uart1_send_len++] = crc;	          /* crc 低字节 */
    uart1_send_data[uart1_send_len++] = crc >> 8;		      /* crc 高字节 */
    uart1_send_flag = 2;
}

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    /* 申请内存空间作为保持寄存器地址,对应功能码03H、06H和10H */
    //PduData.PtrHoldingbase = (uint16_t*)malloc(sizeof(uint16_t) * MB_RTU_DATA_MAX_SIZE);
    PduData.PtrHoldingbase = mbdata;
    MotorDataRefreshInit((uint16_t*)PduData.PtrHoldingbase);
    FillBuf((uint16_t*)PduData.PtrHoldingbase, FUN_CODE_03H);

    /* check preconditions */
    if( ( ucSlaveAddress == MB_ADDRESS_BROADCAST ) ||
        ( ucSlaveAddress < MB_ADDRESS_MIN ) || ( ucSlaveAddress > MB_ADDRESS_MAX ) )
    {
        eStatus = MB_EINVAL;
    }
    else
    {
        ucMBAddress = ucSlaveAddress;

        switch ( eMode )
        {
#if MB_RTU_ENABLED > 0
        case MB_RTU:
            pvMBFrameStartCur = eMBRTUStart;
            pvMBFrameStopCur = eMBRTUStop;
            //peMBFrameSendCur = eMBRTUSend;
            peMBFrameReceiveCur = eMBRTUReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBRTUReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
            pxMBPortCBTimerExpired = xMBRTUTimerT35Expired;

            break;
#endif
#if MB_ASCII_ENABLED > 0
        case MB_ASCII:
            pvMBFrameStartCur = eMBASCIIStart;
            pvMBFrameStopCur = eMBASCIIStop;
            peMBFrameSendCur = eMBASCIISend;
            peMBFrameReceiveCur = eMBASCIIReceive;
            pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL;
            pxMBFrameCBByteReceived = xMBASCIIReceiveFSM;
            pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
            pxMBPortCBTimerExpired = xMBASCIITimerT1SExpired;

            eStatus = eMBASCIIInit( ucMBAddress, ucPort, ulBaudRate, eParity );
            break;
#endif
        default:
            eStatus = MB_EINVAL;
        }

        if( eStatus == MB_ENOERR )
        {
            if( !xMBPortEventInit(  ) )
            {
                /* port dependent event module initalization failed. */
                eStatus = MB_EPORTERR;
            }
            else
            {
                eMBState = STATE_DISABLED;
            }
        }
    }
    return eStatus;
}

#if MB_TCP_ENABLED > 0
eMBErrorCode
eMBTCPInit( USHORT ucTCPPort )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( ( eStatus = eMBTCPDoInit( ucTCPPort ) ) != MB_ENOERR )
    {
        eMBState = STATE_DISABLED;
    }
    else if( !xMBPortEventInit(  ) )
    {
        /* Port dependent event module initalization failed. */
        eStatus = MB_EPORTERR;
    }
    else
    {
        pvMBFrameStartCur = eMBTCPStart;
        pvMBFrameStopCur = eMBTCPStop;
        peMBFrameReceiveCur = eMBTCPReceive;
        peMBFrameSendCur = eMBTCPSend;
        pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL;
        ucMBAddress = MB_TCP_PSEUDO_ADDRESS;
        eMBState = STATE_DISABLED;
    }
    return eStatus;
}
#endif

eMBErrorCode eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}


eMBErrorCode
eMBClose( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        if( pvMBFrameCloseCur != NULL )
        {
            pvMBFrameCloseCur(  );
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBEnable( void )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( eMBState == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        pvMBFrameStartCur(  );
        eMBState = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( void )
{
    eMBErrorCode    eStatus;

    if( eMBState == STATE_ENABLED )
    {
        pvMBFrameStopCur(  );
        eMBState = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( eMBState == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode eMBPoll( void )
{
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static USHORT   usLength;

    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;

    /* Check if the protocol stack is ready. */
    if( eMBState != STATE_ENABLED )
    {
        return MB_EILLSTATE;
    }

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet( &eEvent ) == TRUE )
    {
        switch ( eEvent )
        {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            eStatus = peMBFrameReceiveCur( &ucRcvAddress, &ucMBFrame, &usLength );
            if( eStatus == MB_ENOERR )
            {
                /* Check if the frame is for us. If not ignore the frame. */
                if( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                {//< 数据解析
                    /* 分析数据帧并执行 */
                    uint16_t Ex_code;
                    PduData.Code = ucMBFrame[1];                   // 功能码
                    PduData.Addr = ((ucMBFrame[2] << 8) | ucMBFrame[3]);// 寄存器起始地址
                    PduData.Num = ((ucMBFrame[4] << 8) | ucMBFrame[5]);// 数量(Coil,Input,Holding Reg,Input Reg)
                    PduData.byteNums = ucMBFrame[6];                                     // 获得字节数
                    PduData.ValueReg = (uint8_t*)&ucMBFrame[7];                          // 寄存器值起始地址
                    PduData.PtrHoldingOffset = PduData.PtrHoldingbase + PduData.Addr; // 保持寄存器的起始地址
                    Ex_code = MB_Analyze_Execute();
                    if (Ex_code != EX_CODE_NONE)
                    {/* 出现异常 */
                        MB_Exception_RSP(PduData.Code, Ex_code);
                    }
                    else
                    {
                        MB_RSP(PduData.Code);
                    }
                }
                else
                {
                    //memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // 清空接收缓冲区
                    DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
                    DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USART1_RX_MAXBUFF);
                    DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);     // DMA1 通道5, UART1_RX
                }
            }
            else
            {
                //memset(uart1_recv_data, 0, USART1_RX_MAXBUFF);  // 清空接收缓冲区
                DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);    // DMA1 通道5, UART1_RX
                DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USART1_RX_MAXBUFF);
                DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);     // DMA1 通道5, UART1_RX
            }
            break;
        case EV_EXECUTE:
            break;
        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}

uint16_t* getPDUData(void)
{
    return mbdata;
}
