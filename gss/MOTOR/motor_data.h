#ifndef __motor_data__h
#define __motor_data__h
#include "stdint.h"
extern int Motor_Number;
#define MAX_MOTOR_NUMBER 10
typedef union CONTROLWORD__
{
    struct {
        uint16_t so : 1;													//special operations,特殊操作
        uint16_t ev : 1;													//未知
        uint16_t qs : 1;													//Quick Stop,快速停止指令
        uint16_t eo : 1;													//Emergency Off,紧急停止指令
        uint16_t oms : 3;													//Operation Mode Specific,特定操作模式
        uint16_t fr : 1;													//Fault Reset,故障复位操作
        uint16_t h : 1;														//Halt,停止指令
        uint16_t oms1 : 1;												//Operation Mode Specific1,特定操作模式1
        uint16_t r : 6;														//Reserved,保留位
    }bit;
    uint16_t cd;

}CANOPEN_CONTROLWORD;

typedef union  STATUSWORD__
{
    struct {
        uint16_t rso : 1;													//Reserved for special operations,特殊操作保留位
        uint16_t se : 1;													//Status Error,状态错误标志位
        uint16_t oe : 1;													//Operation Error,操作错误标志位
        uint16_t f : 1;														//Fault,故障标标志位
        uint16_t ve : 1;													//Voltage Error,故障错误标志位
        uint16_t qs : 1;													//Quick Stop,快速停止标志位
        uint16_t sod : 1;													//Switched On Disable,失能时切换开关标志位
        uint16_t w : 1;														//Warning,警告标志位
        uint16_t r : 1;														//Reserved,保留位
        uint16_t rm : 1;													//Remote,远程标志位
        uint16_t oms : 1;													//Operation Mode Specific,特定操作模式标志位
        uint16_t ila : 1;													//Internal Limit Active,内部限制激活标志位
        uint16_t oms1 : 2;												//Operation Mode Specific 1,特定操作模式1标志位
        uint16_t r1 : 2;													//Reserved1,保留位1
    }bit;
    uint16_t sd;

}CANOPEN_STATUSWORD;


typedef union MOTOR_TPDO__
{
    struct
    {
        CANOPEN_STATUSWORD status;  //状态字
        int current_pos;		    //当前位置
        int current_velocity;	    //当前速度
        uint16_t error_code;	    //错误代码 
        uint16_t current_torque;    //当前转矩
        uint8_t heartbeat;      //< 心跳
        uint8_t mapping;	  //< 映射  
    }d;
    uint8_t data[11];

}MOTOR_TPDO;
typedef union MOTOR_RPDO__
{
    struct
    {
        CANOPEN_CONTROLWORD ctrl; //控制字
        CANOPEN_CONTROLWORD ctrl_old; //控制字
        int target_pos;			 //目标位置
        int target_velocity;	 //目标速度
        int target_velocity_old;	 //目标速度
        uint8_t mode;			 //工作模式
        uint16_t target_torque;	 //目标转矩
        uint8_t online : 2;	    //< 在线状态0：掉线；1
        uint8_t mapping : 6;	  //< 映射  
    }d;
    uint8_t data[8];
}MOTOR_RPDO;

extern MOTOR_RPDO mrd[MAX_MOTOR_NUMBER];//< 发送pdo
extern MOTOR_TPDO mtd[MAX_MOTOR_NUMBER];//< 接收pdo

/** Watchdog mode for sync manager configuration.
 *
 * Used to specify, if a sync manager's watchdog is to be enabled.
 */
typedef enum {
    EC_WD_DEFAULT, /**< Use the default setting of the sync manager. */
    EC_WD_ENABLE, /**< Enable the watchdog. */
    EC_WD_DISABLE, /**< Disable the watchdog. */
} ec_watchdog_mode_t;

/** Direction type for PDO assignment functions.
 */
typedef enum {
    EC_DIR_INVALID, /**< Invalid direction. Do not use this value. */
    EC_DIR_OUTPUT, /**< Values written by the master. */
    EC_DIR_INPUT, /**< Values read by the master. */
    EC_DIR_COUNT /**< Number of directions. For internal use only. */
} ec_direction_t;


/** PDO entry configuration information.
 *
 * This is the data type of the \a entries field in ec_pdo_info_t.
 *
 * \see ecrt_slave_config_pdos().
 */
typedef struct {
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    uint8_t bit_length; /**< Size of the PDO entry in bit. */
} ec_pdo_entry_info_t;

/** PDO configuration information.
 *
 * This is the data type of the \a pdos field in ec_sync_info_t.
 *
 * \see ecrt_slave_config_pdos().
 */
typedef struct {
    uint16_t index; /**< PDO index. */
    unsigned int n_entries; /**< Number of PDO entries in \a entries to map.
                              Zero means, that the default mapping shall be
                              used (this can only be done if the slave is
                              present at bus configuration time). */
    ec_pdo_entry_info_t* entries; /**< Array of PDO entries to map. Can either
                                    be \a NULL, or must contain at
                                    least \a n_entries values. */
} ec_pdo_info_t;

/** Sync manager configuration information.
 *
 * This can be use to configure multiple sync managers including the PDO
 * assignment and PDO mapping. It is used as an input parameter type in
 * ecrt_slave_config_pdos().
 */
typedef struct {
    uint8_t index; /**< Sync manager index. Must be less
                     than #EC_MAX_SYNC_MANAGERS for a valid sync manager,
                     but can also be \a 0xff to mark the end of the list. */
    ec_direction_t dir; /**< Sync manager direction. */
    unsigned int n_pdos; /**< Number of PDOs in \a pdos. */
    ec_pdo_info_t* pdos; /**< Array with PDOs to assign. This must contain
                            at least \a n_pdos PDOs. */
    ec_watchdog_mode_t watchdog_mode; /**< Watchdog mode. */
} ec_sync_info_t;

/** List record type for PDO entry mass-registration.
 *
 * This type is used for the array parameter of the
 * ecrt_domain_reg_pdo_entry_list()
 */
typedef struct {
    uint16_t alias; /**< Slave alias address. */
    uint16_t position; /**< Slave position. */
    uint32_t vendor_id; /**< Slave vendor ID. */
    uint32_t product_code; /**< Slave product code. */
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    unsigned int* offset; /**< Pointer to a variable to store the PDO entry's
                       (byte-)offset in the process data. */
    unsigned int* bit_position; /**< Pointer to a variable to store a bit
                                  position (0-7) within the \a offset. Can be
                                  NULL, in which case an error is raised if the
                                  PDO entry does not byte-align. */
} ec_pdo_entry_reg_t;


typedef struct MOTOR_DATA__
{
    struct
    {
        /*配置参数*/
        uint8_t dir : 1;			//< 方向
        uint8_t type : 2;          //< 运动类型 0：旋转；1直线
        uint8_t encoderType : 2;    //< 编码器类型 0：增量；1绝对值
        uint8_t res:3;
        uint8_t origin_signal;           //< 原点信号  0：无
        uint8_t positive_limit_signal;  //< 正极限信号0：无
        uint8_t negative_limit_signal;  //< 负极限信号0：无
        uint16_t max_rpm;       //< 最大转速
        uint32_t encoder_accuracy;       //< 编码器精度
        float origin_pos;       //< 正极限
        float positive_liimit;  //< 正极限
        float negative_liimit;  //< 负极限
        float ratio;			//< 减速比
        float space;			//< 每转距离
        float accel_max;        //< 最大加速度
        float decel_max;        //< 最大减速度
        float a_accel;          //< 加加速
        float a_decel;          //< 减减速


        /*运动参数*/
        uint8_t on:2;	        //< 使能
        uint8_t step:6;         //< 步骤
        uint16_t err_code;      //< 错误码
        uint32_t allt;           //< 单次运动总时间 ms单位
        float run_time;         //< 运动时间片
    }d;
}MOTOR_DATA;

#define MOTOR_STK_SIZE   128 	//任务堆栈大小
#define MOTOR_TASK_PRIO  10   

void MotorDataRefreshInit(uint16_t* pd);

void motor_init_task(void* pvParameters);
void motor_task(void* pvParameters);
#endif

