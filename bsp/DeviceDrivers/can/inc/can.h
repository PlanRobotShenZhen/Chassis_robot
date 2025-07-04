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
 * @file can.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef CAN_H_
#define CAN_H_

#include <rtthread.h>
#include "completion.h"
#include "stdbool.h"

#ifndef RT_CANMSG_BOX_SZ
#define RT_CANMSG_BOX_SZ    16
#endif
#ifndef RT_CANSND_BOX_NUM
#define RT_CANSND_BOX_NUM   1
#endif


#define CAN_RX0_INT_ENABLE 1   //使用中断标识符
#define  CAN_BAUDRATE_1M            1
#define  CAN_BAUDRATE_500K          2
#define  CAN_BAUDRATE_250K          3
#define  CAN_BAUDRATE_125K          4
#define  CAN_BAUDRATE               CAN_BAUDRATE_1M

#if(CAN_BAUDRATE==CAN_BAUDRATE_1M)
#define   CAN_BIT_RSJW              CAN_RSJW_2tq
#define   CAN_BIT_BS1               CAN_TBS1_14tq
#define   CAN_BIT_BS2               CAN_TBS2_3tq
#define   CAN_BAUDRATEPRESCALER     2
#elif(CAN_BAUDRATE==CAN_BAUDRATE_500K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_5tq
#define   CAN_BIT_BS2               CAN_TBS2_2tq
#define   CAN_BAUDRATEPRESCALER     9 
#elif(CAN_BAUDRATE==CAN_BAUDRATE_250K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_4tq
#define   CAN_BIT_BS2               CAN_TBS2_3tq
#define   CAN_BAUDRATEPRESCALER     18
#elif(CAN_BAUDRATE==CAN_BAUDRATE_125K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_13tq
#define   CAN_BIT_BS2               CAN_TBS2_4tq
#define   CAN_BAUDRATEPRESCALER     16
#endif


#define  CAN_FILTERNUM0                 ((uint8_t)0)
#define  CAN_FILTER_STDID(STDID)        ((STDID&0x7FF)<<5)
#define  CAN_STD_ID_H_MASK_DONT_CARE    0x0000
#define  CAN_STD_ID_L_MASK_DONT_CARE    0x0000

extern int g_nMotorLF_config;
extern int g_nMotorLB_config;
extern int g_nMotorRF_config;
extern int g_nMotorRB_config;  //左右两侧电机配置变量


extern int g_nReInitMotor_LB;   //处理驱动器断电而开发板未断电，重新初始化驱动器的问题
extern int g_nReInitMotor_LF;   //为1代表需要重新初始化，为0则代表不需要重新初始化
extern int g_nReInitMotor_RF;
extern int g_nReInitMotor_RB;


extern int g_nHeart_count_LB;    //设置了心跳包，在规定次数内没产生心跳，则认为从机下线
extern int g_nHeart_count_LF;    //将其状态设置为STATE_STOP
extern int g_nHeart_count_RF;
extern int g_nHeart_count_RB;

extern int g_nHeart_Lastcount_LB;  //记录上次进入心跳包的计数值
extern int g_nHeart_Lastcount_LF;
extern int g_nHeart_Lastcount_RF;
extern int g_nHeart_Lastcount_RB;


//定义存放反馈速度的共用体
union URcv_Vel_Data {
    int nVelcity;
    unsigned char ucData[4];
};

// 定义驱动器故障接收的共用体
union URcv_ERROR_Data {
    unsigned short usError;
    unsigned char ucData[2];
};


//定义心跳包状态
enum ENUM_HEART_STAT
{
    STATE_UNKNOW = 0x00,   //未知状态
    STATE_STOP = 0x04,		//停止状态
    STATE_START = 0x05,		//操作状态
    STATE_PRE_OPERATE = 0x7F		//预操作状态
};


extern union URcv_Vel_Data uVelLF;
extern union URcv_Vel_Data uVelLB;
extern union URcv_Vel_Data uVelRF;
extern union URcv_Vel_Data uVelRB;  //存放左右实时速度的变量

extern union URcv_ERROR_Data uError; // 接收驱动器的故障状态

extern enum ENUM_HEART_STAT eLF_Motor_Heart;
extern enum ENUM_HEART_STAT eLB_Motor_Heart;
extern enum ENUM_HEART_STAT eRF_Motor_Heart;
extern enum ENUM_HEART_STAT eRB_Motor_Heart;   //左右电机心跳包

extern enum ENUM_ERROR_STATE eLF_Motor_Error;
extern enum ENUM_ERROR_STATE eLB_Motor_Error;
extern enum ENUM_ERROR_STATE eRF_Motor_Error;
extern enum ENUM_ERROR_STATE eRB_Motor_Error;   // 驱动器故障码读取
enum CANBAUD
{
    CAN1MBaud   = 1000UL * 1000,/* 1 MBit/sec   */
    CAN500kBaud = 1000UL * 500, /* 500 kBit/sec */
    CAN250kBaud = 1000UL * 250, /* 250 kBit/sec */
    CAN125kBaud = 1000UL * 125, /* 125 kBit/sec */
    CAN100kBaud = 1000UL * 100, /* 100 kBit/sec */
    CAN50kBaud  = 1000UL * 50,  /* 50 kBit/sec  */
    CAN20kBaud  = 1000UL * 20,  /* 20 kBit/sec  */
    CAN10kBaud  = 1000UL * 10   /* 10 kBit/sec  */
};

#define RT_CAN_MODE_NORMAL              0
#define RT_CAN_MODE_LISEN               1
#define RT_CAN_MODE_LOOPBACK            2
#define RT_CAN_MODE_LOOPBACKANLISEN     3

#define RT_CAN_MODE_PRIV                0x01
#define RT_CAN_MODE_NOPRIV              0x00

struct rt_can_filter_item
{
    rt_uint32_t id  : 29;
    rt_uint32_t ide : 1;
    rt_uint32_t rtr : 1;
    rt_uint32_t mode : 1;
    rt_uint32_t mask;
    rt_int32_t hdr;
#ifdef RT_CAN_USING_HDR
    rt_err_t (*ind)(rt_device_t dev, void *args , rt_int32_t hdr, rt_size_t size);
    void *args;
#endif /*RT_CAN_USING_HDR*/
};

#ifdef RT_CAN_USING_HDR
#define RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,ind,args) \
     {(id), (ide), (rtr), (mode), (mask), -1, (ind), (args)}
#define RT_CAN_FILTER_STD_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,0,0,0,0xFFFFFFFF,ind,args)
#define RT_CAN_FILTER_EXT_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,1,0,0,0xFFFFFFFF,ind,args)
#define RT_CAN_STD_RMT_FILTER_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,0,1,0,0xFFFFFFFF,ind,args)
#define RT_CAN_EXT_RMT_FILTER_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,1,1,0,0xFFFFFFFF,ind,args)
#define RT_CAN_STD_RMT_DATA_FILTER_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,0,0,1,0xFFFFFFFF,ind,args)
#define RT_CAN_EXT_RMT_DATA_FILTER_INIT(id,ind,args) \
     RT_CAN_FILTER_ITEM_INIT(id,1,0,1,0xFFFFFFFF,ind,args)
#else

#define RT_CAN_FILTER_ITEM_INIT(id,ide,rtr,mode,mask,ind,args) \
     {(id), (ide), (rtr), (mode), (mask), -1, (ind), (args)}
#define RT_CAN_FILTER_STD_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,0,0,0,0xFFFFFFFF)
#define RT_CAN_FILTER_EXT_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,1,0,0,0xFFFFFFFF)
#define RT_CAN_STD_RMT_FILTER_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,0,1,0,0xFFFFFFFF)
#define RT_CAN_EXT_RMT_FILTER_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,1,1,0,0xFFFFFFFF)
#define RT_CAN_STD_RMT_DATA_FILTER_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,0,0,1,0xFFFFFFFF)
#define RT_CAN_EXT_RMT_DATA_FILTER_INIT(id) \
     RT_CAN_FILTER_ITEM_INIT(id,1,0,1,0xFFFFFFFF)
#endif

struct rt_can_filter_config
{
    rt_uint32_t count;
    rt_uint32_t actived;
    struct rt_can_filter_item *items;
};

struct can_configure
{
    rt_uint32_t baud_rate;
    rt_uint32_t msgboxsz;
    rt_uint32_t sndboxnumber;
    rt_uint32_t mode      : 8;
    rt_uint32_t privmode  : 8;
    rt_uint32_t reserved  : 16;
    rt_uint32_t ticks;
#ifdef RT_CAN_USING_HDR
    rt_uint32_t maxhdr;
#endif
};

#define CANDEFAULTCONFIG \
{\
        CAN500kBaud,\
        RT_CANMSG_BOX_SZ,\
        RT_CANSND_BOX_NUM,\
        RT_CAN_MODE_LOOPBACK,\
};

struct rt_can_ops;
#define RT_CAN_CMD_SET_FILTER       0x13
#define RT_CAN_CMD_SET_BAUD         0x14
#define RT_CAN_CMD_SET_MODE         0x15
#define RT_CAN_CMD_SET_PRIV         0x16
#define RT_CAN_CMD_GET_STATUS       0x17
#define RT_CAN_CMD_SET_STATUS_IND   0x18
#define RT_CAN_CMD_SET_BUS_HOOK     0x19

#define RT_DEVICE_CAN_INT_ERR       0x1000

enum RT_CAN_STATUS_MODE
{
    NORMAL = 0,
    ERRWARNING = 1,
    ERRPASSIVE = 2,
    BUSOFF = 4,
};
enum RT_CAN_BUS_ERR
{
    RT_CAN_BUS_NO_ERR = 0,
    RT_CAN_BUS_BIT_PAD_ERR = 1,
    RT_CAN_BUS_FORMAT_ERR = 2,
    RT_CAN_BUS_ACK_ERR = 3,
    RT_CAN_BUS_IMPLICIT_BIT_ERR = 4,
    RT_CAN_BUS_EXPLICIT_BIT_ERR = 5,
    RT_CAN_BUS_CRC_ERR = 6,
};

struct rt_can_status
{
    rt_uint32_t rcverrcnt;
    rt_uint32_t snderrcnt;
    rt_uint32_t errcode;
    rt_uint32_t rcvpkg;
    rt_uint32_t dropedrcvpkg;
    rt_uint32_t sndpkg;
    rt_uint32_t dropedsndpkg;
    rt_uint32_t bitpaderrcnt;
    rt_uint32_t formaterrcnt;
    rt_uint32_t ackerrcnt;
    rt_uint32_t biterrcnt;
    rt_uint32_t crcerrcnt;
    rt_uint32_t rcvchange;
    rt_uint32_t sndchange;
    rt_uint32_t lasterrtype;
};

#ifdef RT_CAN_USING_HDR
struct rt_can_hdr
{
    rt_uint32_t connected;
    rt_uint32_t msgs;
    struct rt_can_filter_item filter;
    struct rt_list_node list;
};
#endif
struct rt_can_device;
typedef rt_err_t (*rt_canstatus_ind)(struct rt_can_device *, void *);
typedef struct rt_can_status_ind_type
{
    rt_canstatus_ind ind;
    void *args;
} *rt_can_status_ind_type_t;
typedef void (*rt_can_bus_hook)(struct rt_can_device *);
struct rt_can_device
{
    struct rt_device parent;

    const struct rt_can_ops *ops;
    struct can_configure config;
    struct rt_can_status status;

    rt_uint32_t timerinitflag;
    struct rt_timer timer;

    struct rt_can_status_ind_type status_indicate;
#ifdef RT_CAN_USING_HDR
    struct rt_can_hdr *hdr;
#endif
#ifdef RT_CAN_USING_BUS_HOOK
    rt_can_bus_hook bus_hook;
#endif /*RT_CAN_USING_BUS_HOOK*/
#ifdef RT_USING_MUTEX
    struct rt_mutex lock;
#endif

    void *can_rx;
    void *can_tx;
};
typedef struct rt_can_device *rt_can_t;

#define RT_CAN_STDID 0
#define RT_CAN_EXTID 1
#define RT_CAN_DTR   0
#define RT_CAN_RTR   1

typedef struct rt_can_status *rt_can_status_t;
struct rt_can_msg
{
    rt_uint32_t id  : 29;
    rt_uint32_t ide : 1;
    rt_uint32_t rtr : 1;
    rt_uint32_t rsv : 1;
    rt_uint32_t len : 8;
    rt_uint32_t priv : 8;
    rt_int32_t hdr : 8;
    rt_uint32_t reserved : 8;
    rt_uint8_t data[8];
};
typedef struct rt_can_msg *rt_can_msg_t;

struct rt_can_msg_list
{
    struct rt_list_node list;
#ifdef RT_CAN_USING_HDR
    struct rt_list_node hdrlist;
    struct rt_can_hdr *owner;
#endif
    struct rt_can_msg data;
};

struct rt_can_rx_fifo
{
    /* software fifo */
    struct rt_can_msg_list *buffer;
    rt_uint32_t freenumbers;
    struct rt_list_node freelist;
    struct rt_list_node uselist;
};

#define RT_CAN_SND_RESULT_OK        0
#define RT_CAN_SND_RESULT_ERR       1
#define RT_CAN_SND_RESULT_WAIT      2

#define RT_CAN_EVENT_RX_IND         0x01    /* Rx indication */
#define RT_CAN_EVENT_TX_DONE        0x02    /* Tx complete   */
#define RT_CAN_EVENT_TX_FAIL        0x03    /* Tx fail   */
#define RT_CAN_EVENT_RX_TIMEOUT     0x05    /* Rx timeout    */
#define RT_CAN_EVENT_RXOF_IND       0x06    /* Rx overflow */

struct rt_can_sndbxinx_list
{
    struct rt_list_node list;
    struct rt_completion completion;
    rt_uint32_t result;
};

struct rt_can_tx_fifo
{
    struct rt_can_sndbxinx_list *buffer;
    struct rt_semaphore sem;
    struct rt_list_node freelist;
};

struct rt_can_ops
{
    rt_err_t (*configure)(struct rt_can_device *can, struct can_configure *cfg);
    rt_err_t (*control)(struct rt_can_device *can, int cmd, void *arg);
    int (*sendmsg)(struct rt_can_device *can, const void *buf, rt_uint32_t boxno);
    int (*recvmsg)(struct rt_can_device *can, void *buf, rt_uint32_t boxno);
};

rt_err_t rt_hw_can_register(struct rt_can_device    *can,
                            const char              *name,
                            const struct rt_can_ops *ops,
                            void                    *data);
void rt_hw_can_isr(struct rt_can_device *can, int event);
#endif /*_CAN_H*/

