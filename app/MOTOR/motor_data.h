#ifndef __motor_data__h
#define __motor_data__h
#include "stdint.h"
#include "stdbool.h"
#define MAX_MOTOR_NUMBER 10
#define TPDO_GROUP_COUNT 3

typedef union CONTROLWORD__
{
    struct {		
		uint16_t bit0 : 1;											
    uint16_t bit1 : 1;									
    uint16_t bit2 : 1;
    uint16_t bit3 : 1;											
    uint16_t bit4 : 1;									
    uint16_t bit5 : 1;												
    uint16_t bit6 : 1;												
    uint16_t bit7 : 1;											
    uint16_t bit8 : 1;											
    uint16_t bit9 : 1;													
    uint16_t bitA : 1;													
    uint16_t bitB : 1;												
    uint16_t bitC : 1;												
    uint16_t bitD : 1;													
    uint16_t bitE : 1;												
    uint16_t bitF : 1;				
    }bit;
    uint16_t cw;

}CANOPEN_CONTROLWORD;

typedef union STATUSWORD__
{
    struct {
		uint16_t bit0 : 1;											
    uint16_t bit1 : 1;									
    uint16_t bit2 : 1;
    uint16_t bit3 : 1;											
    uint16_t bit4 : 1;									
    uint16_t bit5 : 1;												
    uint16_t bit6 : 1;												
    uint16_t bit7 : 1;											
    uint16_t bit8 : 1;											
    uint16_t bit9 : 1;													
    uint16_t bitA : 1;													
    uint16_t bitB : 1;												
    uint16_t bitC : 1;												
    uint16_t bitD : 1;													
    uint16_t bitE : 1;												
    uint16_t bitF : 1;													
    }bit;
    uint16_t sw;

}CANOPEN_STATUSWORD;

typedef union MOTOR_TPDO__
{
    struct
    {
        CANOPEN_STATUSWORD status;						//״̬�֣�2�ֽڣ�
        int current_pos;								//��ǰλ�ã�4�ֽڣ�	
        uint16_t error_code;							//�������
        uint16_t none_count;	        				//��TPDO��Ӧ������1�μ��20ms
        bool online;	        					    //����״̬ 1�����ߣ�0������
    }d;
    uint8_t data[8];
}MOTOR_TPDO;

typedef union MOTOR_RPDO__
{
    struct
    {
        CANOPEN_CONTROLWORD ctrl;						//������(16λ��
        int target_pos_vel;								//Ŀ��λ��/�ٶ�
        uint8_t mode;									//����ģʽ
        uint16_t target_torque;							//Ŀ��ת��
    }d;
    uint8_t data[8];
}MOTOR_RPDO;

extern MOTOR_RPDO mrd[MAX_MOTOR_NUMBER];//< ����pdo��ֻ�õ�����״̬��������TPDOMessage
extern MOTOR_TPDO mtd[MAX_MOTOR_NUMBER];//< ����pdo

typedef union MOTOR_PARA__
{
    struct
    {
		uint8_t type;										    //����ͺ�	
		uint8_t sportmode;										//�˶�ģʽ
		uint8_t enstate;										//���ʹ��״̬	
		uint8_t canid;											//CAN_ID
    }d;
    uint8_t data[4];
}MOTOR_PARA;
extern MOTOR_PARA motor_para[MAX_MOTOR_NUMBER];

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
        /*���ò���*/
        uint8_t dir : 1;			//< ����
        uint8_t type : 2;          //< �˶����� 0����ת��1ֱ��
        uint8_t encoderType : 2;    //< ���������� 0��������1����ֵ
        uint8_t res:3;
        uint8_t origin_signal;           //< ԭ���ź�  0����
        uint8_t positive_limit_signal;  //< �������ź�0����
        uint8_t negative_limit_signal;  //< �������ź�0����
        uint16_t max_rpm;       //< ���ת��
        uint32_t encoder_accuracy;       //< ����������
        float origin_pos;       //< ������
        float positive_liimit;  //< ������
        float negative_liimit;  //< ������
        float ratio;			//< ���ٱ�
        float space;			//< ÿת����
        float accel_max;        //< �����ٶ�
        float decel_max;        //< �����ٶ�
        float a_accel;          //< �Ӽ���
        float a_decel;          //< ������


        /*�˶�����*/
        uint8_t on:2;	        //< ʹ��
        uint8_t step:6;         //< ����
        uint16_t err_code;      //< ������
        uint32_t allt;           //< �����˶���ʱ�� ms��λ
        float run_time;         //< �˶�ʱ��Ƭ
    }d;
}MOTOR_DATA;

#define MOTOR_STK_SIZE   128 	//�����ջ��С
#define MOTOR_TASK_PRIO  10   

void Motor_task(void* pvParameters);

#endif

