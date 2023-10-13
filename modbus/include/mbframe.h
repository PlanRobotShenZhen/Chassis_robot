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
 * File: $Id: mbframe.h,v 1.9 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_FRAME_H
#define _MB_FRAME_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#include "stm32f10x.h"
/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_SIZE_MAX     253 /*!< Maximum size of a PDU. */
#define MB_PDU_SIZE_MIN     1   /*!< Function Code */
#define MB_PDU_FUNC_OFF     0   /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF     1   /*!< Offset for response data in PDU. */
/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
    __IO uint8_t  Code;  	        // 功能码
    __IO uint8_t byteNums; 	        // 字节数
    __IO uint16_t Addr;            // 操作内存的起始地址
    __IO uint16_t Num; 	            // 寄存器或者线圈的数量
    __IO uint16_t _CRC;       	      // CRC校验码
    __IO uint8_t* ValueReg; 	      // 10H功能码的数据
    __IO uint16_t* PtrHoldingbase;  // HoldingReg内存首地址
    __IO uint16_t* PtrHoldingOffset;// HoldingReg内存首地址
}PDUData_TypeDef;
typedef struct
{
    uint16_t IN1;

    /* 01H 05H 读写单个强制线圈 */
    uint16_t D01;
    uint16_t D02;
    uint16_t D03;
    uint16_t D04;

}REG_VALUE;
/* 宏定义 --------------------------------------------------------------------*/
#define MB_SLAVEADDR            0x0001
#define MB_ALLSLAVEADDR         0x00FF

#define FUN_CODE_01H            0x01  // 功能码01H 
#define FUN_CODE_02H            0x02  // 功能码02H
#define FUN_CODE_03H            0x03  // 功能码03H
#define FUN_CODE_04H            0x04  // 功能码04H
#define FUN_CODE_05H            0x05  // 功能码05H
#define FUN_CODE_06H            0x06  // 功能码06H
#define FUN_CODE_10H            0x10  // 功能码10H
#define FUN_CODE_AAH            0xAA  // 功能码AAH
/* 本例程所支持的功能码,需要添加新功能码还需要在.c文件里面添加 */
#define IS_NOT_FUNCODE(code)  (!((code == FUN_CODE_01H)||\
                                 (code == FUN_CODE_02H)||\
                                 (code == FUN_CODE_03H)||\
                                 (code == FUN_CODE_04H)||\
                                 (code == FUN_CODE_05H)||\
                                 (code == FUN_CODE_06H)||\
                                 (code == FUN_CODE_AAH)||\
                                 (code == FUN_CODE_10H)))

#define EX_CODE_NONE           0x00  // 异常码 无异常
#define EX_CODE_01H            0x01  // 异常码
#define EX_CODE_02H            0x02  // 异常码
#define EX_CODE_03H            0x03  // 异常码
#define EX_CODE_04H            0x04  // 异常码

#define COIL_D01		0x01
#define COIL_D02		0x02
#define COIL_D03		0x03
#define COIL_D04		0x04

#define REG_IN1		  0x0020
/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
#define HOLD_REG_01		0x0010
#define HOLD_REG_02		0x0011
#define HOLD_REG_03		0x0012

/* 扩展变量 ------------------------------------------------------------------*/
extern PDUData_TypeDef PduData;




/* ----------------------- Prototypes  0-------------------------------------*/
typedef void    ( *pvMBFrameStart ) ( void );

typedef void    ( *pvMBFrameStop ) ( void );

typedef eMBErrorCode( *peMBFrameReceive ) ( UCHAR * pucRcvAddress,
                                            UCHAR ** pucFrame,
                                            USHORT * pusLength );

typedef eMBErrorCode( *peMBFrameSend ) ( UCHAR slaveAddress,
                                         const UCHAR * pucFrame,
                                         USHORT usLength );

typedef void( *pvMBFrameClose ) ( void );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
