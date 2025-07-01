#ifndef __485_ADDRESS__h
#define __485_ADDRESS__h

enum address_485 {
    // RO参数
    // 小车参数
    car_type = 0,
    car_product_number,
    car_version,
    car_length,
    car_width,
    car_height,
    car_wheelbase, //机器人轴距
    car_tread,//机器人轮距
    car_ground_clearance,
    wheel_radius,//车轮半径
    gross_max,
    rated_load,
    motor_number,
    driving_method,
    ros_chargingcommand,
    car_running_state = 15,
    car_error_messages,
    car_light_messages,
    linear_speed_feedback,//10-3m/s
    yaw_speed_feedback,
    degree_feedback,
    exio_input_status,
    BatteryManufacturer,//< 电池厂家
    BatteryStatus,
    BatteryQuantity,
    BatteryVoltage,
    BatteryCurrent,
    BatteryTemperature,
    BatteryHealth,
    BatteryProtectStatus,
    BatteryChargingTime,
    BatteryAvailabilityTime,
    Odom1ForRobot,
    Odom2ForRobot,
    TpdoGroupCount,//TPDO组数
    // 电机参数
    motor1_type = 35,//电机厂商及型号
    motor1_state_word,
    motor1_temperature,
    motor1_sport_mode,
    motor1_enable_state,
    motor1_node_state,
    motor1_error_code,
    motor1_position1_feedback,
    motor1_position2_feedback,
    motor1_rpm_feedback,//反馈角速度0.1rpm
    motor1_torgue_feedback,
    motor1_voltage,
    motor1_current,
    undefined48,  
    undefined49,  
    motor2_type = 50,
    motor2_state_word,
    motor2_temperature,
    motor2_sport_mode,
    motor2_enable_state,
    motor2_node_state,
    motor2_error_code,
    motor2_position1_feedback,
    motor2_position2_feedback,
    motor2_rpm_feedback,
    motor2_torgue_feedback,
    motor2_voltage,
    motor2_current,
    undefined63,
    undefined64,
    motor3_type = 65,
    motor3_state_word,
    motor3_temperature,
    motor3_sport_mode,
    motor3_enable_state,
    motor3_node_state,
    motor3_error_code,
    motor3_position1_feedback,
    motor3_position2_feedback,
    motor3_rpm_feedback,
    motor3_torgue_feedback,
    motor3_voltage,
    motor3_current,
    undefined78,  
    undefined79,  
    motor4_type = 80,
    motor4_state_word,
    motor4_temperature,
    motor4_sport_mode,
    motor4_enable_state,
    motor4_node_state,
    motor4_error_code,
    motor4_position1_feedback,
    motor4_position2_feedback,
    motor4_rpm_feedback,
    motor4_torgue_feedback,
    motor4_voltage,
    motor4_current,
    undefined93,
    undefined94,
    motor5_type = 95,
    motor5_state_word,
    motor5_temperature,
    motor5_sport_mode,
    motor5_enable_state,
    motor5_node_state,
    motor5_error_code,
    motor5_position1_feedback,
    motor5_position2_feedback,
    motor5_rpm_feedback,
    motor5_torgue_feedback,
    motor5_voltage,
    motor5_current,
    undefined108,  
    undefined109,  
    motor6_type = 110,
    motor6_state_word,
    motor6_temperature,
    motor6_sport_mode,
    motor6_enable_state,
    motor6_node_state,
    motor6_error_code,
    motor6_position1_feedback,
    motor6_position2_feedback,
    motor6_rpm_feedback,
    motor6_torgue_feedback,
    motor6_voltage,
    motor6_current,
    undefined123,  
    undefined124,  
    motor7_type = 125,
    motor7_state_word,
    motor7_temperature,
    motor7_sport_mode,
    motor7_enable_state,
    motor7_node_state,
    motor7_error_code,
    motor7_position1_feedback,
    motor7_position2_feedback,
    motor7_rpm_feedback,
    motor7_torgue_feedback,
    motor7_voltage,
    motor7_current,
    undefined138,  
    undefined139,  
    motor8_type = 140,
    motor8_state_word,
    motor8_temperature,
    motor8_sport_mode,
    motor8_enable_state,
    motor8_node_state,
    motor8_error_code,
    motor8_position1_feedback,
    motor8_position2_feedback,
    motor8_rpm_feedback,
    motor8_torgue_feedback,
    motor8_voltage,
    motor8_current,
    undefined153,
    undefined154,

    //RC小车参数
    rc_car_speed = 155,
    rc_car_mileage_high,
    rc_car_mileage_low,
    //超声波
    Ultrasonic1,
    Ultrasonic2,
    // 航模参数
    rc_connect_state = 160,//航模接收成功
    rc_ch1_value,
    rc_ch2_value,
    rc_ch3_value,
    rc_ch4_value,
    rc_ch5_value,
    rc_ch6_value,
    rc_ch7_value,
    rc_ch8_value,
    rc_ch9_value,
    rc_ch10_value,
    rc_ch11_value,
    rc_ch12_value,
    rc_ch13_value,
    rc_ch14_value,
    rc_ch15_value,
    rc_ch16_value,
    rc_min_value,
    rc_base_value,
    rc_max_value,
    rc_gears_difference,//拨杆半量程
    // adc
    adc1_jt_value = 181,
    adc2_value,
    adc_yl_1,
    adc_yl_2,
    adc_yl_3,
    adc_yl_4,
    adc_yl_5,
    MT_CURR_Value,
    IOUT_24VARM_Value,
    IOUT_19V_Value,
    IOUT_5V_Value,
    IOUT_12VPC_Value,
    rc_encoder_high,
    rc_encoder_low,
    // RW参数
    // 小车参数
    CAN_baud = 195,
    moddbus_485_id,
    moddbus_485_baud,
    control_mode,
    light_control,
    power_control,
    software_reset,
    error_clearance,
    can_reinitialize,//CAN总线初始化s
    para_save,

    robot_acceleration,
    robot_forward_direction,	//前进方向
    robot_turning_direction,	//转弯方向
    max_linear_speed,
    max_yaw_speed,
    max_angle,
    target_linear_speed,//目标线速度（10-3m/s)
    target_yaw_speed,//目标角速度（10-3rad/s)
    target_angle,
    linear_low,
    linear_middle,
    linear_high,
    angular_low,
    angular_middle,
    angular_high,

    // 电机参数
	motor1_CAN_id = 220,
    motor1_reduction_ratio,//减速比
    motor1_direction,
    motor1_acceleration_time, 
    motor1_deceleration_time, 
    motor1_target_position1, 
    motor1_target_position2, 
    motor1_target_rpm, //1rpm
    motor1_target_torque,
    motor1_profile_rpm,    
    motor1_profile_acce,
    motor1_profile_jerk,
    undefined232, 
    undefined233, 
    undefined234,
	motor2_CAN_id = 235,
    motor2_reduction_ratio,
    motor2_direction,
    motor2_acceleration_time, 
    motor2_deceleration_time, 
    motor2_target_position1, 
    motor2_target_position2, 
    motor2_target_rpm, 
    motor2_target_torque,
    motor2_profile_rpm,    
    motor2_profile_acce,
    motor2_profile_jerk,
    undefined247, 
    undefined248, 
    undefined249,
	motor3_CAN_id = 250,
    motor3_reduction_ratio,
    motor3_direction,
    motor3_acceleration_time, 
    motor3_deceleration_time, 
    motor3_target_position1, 
    motor3_target_position2, 
    motor3_target_rpm, 
    motor3_target_torque,
    motor3_profile_rpm,    
    motor3_profile_acce,
    motor3_profile_jerk,
    undefined262, 
    undefined263, 
    undefined264,
	motor4_CAN_id = 265,
    motor4_reduction_ratio,
    motor4_direction,
    motor4_acceleration_time, 
    motor4_deceleration_time, 
    motor4_target_position1, 
    motor4_target_position2, 
    motor4_target_rpm,  
    motor4_target_torque,  
    motor4_profile_rpm,    
    motor4_profile_acce,
    motor4_profile_jerk,
    undefined277, 
    undefined278, 
    undefined279,
	motor5_CAN_id = 280,
    motor5_reduction_ratio,
    motor5_direction,
    motor5_acceleration_time, 
    motor5_deceleration_time, 
    motor5_target_position1, 
    motor5_target_position2, 
    motor5_target_rpm, 
    motor5_target_torque,
    motor5_profile_rpm,    
    motor5_profile_acce,
    motor5_profile_jerk,
    undefined292, 
    undefined293, 
    undefined294,
	motor6_CAN_id = 295,
    motor6_reduction_ratio,
    motor6_direction,
    motor6_acceleration_time, 
    motor6_deceleration_time, 
    motor6_target_position1, 
    motor6_target_position2, 
    motor6_target_rpm, 
    motor6_target_torque,
    motor6_profile_rpm,    
    motor6_profile_acce,
    motor6_profile_jerk,
    undefined307, 
    undefined308, 
    undefined309,
	motor7_CAN_id = 310,
    motor7_reduction_ratio,
    motor7_direction,
    motor7_acceleration_time, 
    motor7_deceleration_time, 
    motor7_target_position1, 
    motor7_target_position2, 
    motor7_target_rpm, 
    motor7_target_torque,
    motor7_profile_rpm,    
    motor7_profile_acce,
    motor7_profile_jerk,
    undefined322, 
    undefined323, 
    undefined324,
	motor8_CAN_id = 325,
    motor8_reduction_ratio,
    motor8_direction,
    motor8_acceleration_time, 
    motor8_deceleration_time, 
    motor8_target_position1, 
    motor8_target_position2, 
    motor8_target_rpm, 
    motor8_target_torque,  
    motor8_profile_rpm,    
    motor8_profile_acce,
    motor8_profile_jerk,
    undefined337, 
    undefined338, 
    undefined339,
    ro_motor_gap = 340,//电机类型的地址差值
    rw_motor_gap,//电机CANid的地址差值
    torque_cofficient,//力矩档位，1-3
    undefined343, 
    undefined344,    
    // 航模参数
    virtually_rc_ch1_value = 345,
    virtually_rc_ch2_value,
    virtually_rc_ch3_value,
    virtually_rc_ch4_value,
    virtually_rc_ch5_value,
    virtually_rc_ch6_value,
    virtually_rc_ch7_value,
    virtually_rc_ch8_value,
    virtually_rc_ch9_value,
    virtually_rc_ch10_value,
    virtually_rc_ch11_value,
    virtually_rc_ch12_value,
    virtually_rc_ch13_value,
    virtually_rc_ch14_value,
    virtually_rc_ch15_value,
    virtually_rc_ch16_value,
    undefined361,
    undefined362, 
    undefined363,
    undefined364,
    rc_encoder_reset = 365,//< 该参数写5，脉冲计数和编码器清零
    rc_encoder_dir,//< 0:按编码器方向加减;1:脉冲计数只累加
    rc_ratio,//< 减速比，精度0.001
    rc_encoder_accuracy,//< 编码器精度，精度1
    rc_tire_diameter,//< 轮胎直径，单位mm
    rc_speed_shifting,//< RC速度偏移
    rc_angle_shifting,//< RC转向偏移
    rc_magnification,//< RC速度倍率
    ir_functioncode,//红外功能码：1：对齐成功；2：对齐失败；
    undefined374,   
    Middle_battery_threshold = 375,
    Low_battery_threshold, 
    Power_board_version,
	motor1_pulse_num,
	motor1_pulse_num1,
	motor2_pulse_num,
	motor2_pulse_num1,
	motor3_pulse_num,
	motor3_pulse_num1,
	motor4_pulse_num,
	motor4_pulse_num1,	
};


//驱动方式
enum enum_driving_method
{
    front_drive,   //前驱
    rear_drive,    //后驱
    full_drive,   //全驱
};

//电池厂家
enum enum_battery_manufacturer
{
    batterydefault,//默认
    lishen,//锂神  
};

//驱动类型
enum enum_motor_type
{
    servo_zlac=0,//< 中菱伺服
    servo_wanze,//< 万泽伺服
    servo_zlacd,//< 中菱伺服一拖二驱动
    servo_plan,//< 普蓝伺服
};

//运行状态
enum enum_car_running_state
{
    car_standby,   //待机
    car_running,    //运行
    car_error,   //故障
    car_emergency_stop,   //急停
};

//机器人故障信息
enum enum_car_error_information
{
    car_normal,             //正常
    single_motor_error,     //单电机故障
    multiple_motor_errors,  //多电机故障
    spi_error,              //spi故障
};

//运动模式
enum enum_sport_mode
{
	position_mode,    //位置模式
	speed_mode,    // 速度模式
	torque_mode,    // 力矩模式
};
extern enum enum_sport_mode sport_mode;

// 电机使能状态
enum enum_enable_state
{
	disable_state,   // 失能
	enable_state,    // 使能
};

// 电机节点状态
enum enum_node_state
{
    node_standby,   		//待机
    node_running,   		//运行
    node_error,   			//故障
    node_disconnect,   	    //掉线
};

// 定义驱动器的故障状态
enum enum_error_state
{
    error_none = 0x00,  // 无错误
    error_over_value = 0x01,  // 过压
    error_less_value = 0x02,  // 欠压
    error_over_current = 0x04,  // 过流
    error_over_load = 0x08,  // 过载
    error_overdiff_current = 0x10,  // 电流超差
    error_overdiff_encoder = 0x20,  // 编码器超差
    error_overdiff_velocity = 0x40,  // 速度超差
    error_ref_value = 0x80,  // 参考电压出错
    error_eeprom = 0x100, // EEPROM读写错误
    error_hall = 0x200  // 霍尔出错
};

// 枚举定义小车的控制方式
enum enum_control_mode
{
	control_mode_unknown,    // 未知控制模式
	control_mode_remote,    // 航模控制方式
	control_mode_ros,    // ROS控制模式
	control_mode_ipc,    	// 上位机控制模式
	control_mode_other,     // 其他控制模式
};

// 机器人灯光控制
enum enum_light_state
{
	light_off,    // 车灯关闭
	light_gleam,    // 车灯闪烁
	light_on,    // 车灯常亮
	light_emergency,    // 急停
};

// 软件复位重启
enum enum_software_reset
{
	none_reset,    // 正常
	once_reset,    // 软件重启
};

// 手柄连接状态
enum enum_rc_state
{
	rc_state_failed,  	// 未连接
	rc_state_success,  	// 连接
};

// CAN波特率
enum enum_can_baud
{
	can_baud_1000000,  	// 1M
	can_baud_500000,  	// 500K
};

// 串口波特率
enum enum_usart_baud
{
	usart_baud_9600,  	// 9600
	usart_baud_19200,   // 19200
	usart_baud_57600,   // 57600
	usart_baud_115200,  // 115200
	usart_baud_256000,  // 256000
	usart_baud_512000,  // 512000
	usart_baud_921600,  // 921600  
	usart_baud_1000000, // 1M   
	usart_baud_1500000, // 1.5M
	usart_baud_2000000, // 2M
};

// 一些设置的数值
enum enum_setting_values
{
	rc_encoder_clear = 5,  	        //脉冲计数和编码器清零
	rc_encoder_signed = 0,          //按编码器方向加减
	rc_encoder_unsigned = 1,        //脉冲计数只累加   
    car_software_reset = 0xA5,      //软件复位一次  
    car_error_clearance = 1,        //故障清除
    car_can_reinit = 5,             //can重新初始化
    car_direct_forward = 1,         //正转
    car_direct_revert = -1,         //反转
    power_board_version_old = 0,    //旧电源板
    power_board_version_new = 1,    //新电源板
};



#endif
