#ifndef __485_ADDRESS__h
#define __485_ADDRESS__h

enum address_485 {
    // RO����
    // С������
    car_type = 0,
    car_product_number,
    car_version,
    car_length,
    car_width,
    car_height,
    car_wheelbase, //���������
    car_tread,//�������־�
    car_ground_clearance,
    wheel_radius,//���ְ뾶
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
    BatteryManufacturer,//< ��س���
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
    TpdoGroupCount,//TPDO����
    // �������
    motor1_type = 35,//������̼��ͺ�
    motor1_state_word,
    motor1_temperature,
    motor1_sport_mode,
    motor1_enable_state,
    motor1_node_state,
    motor1_error_code,
    motor1_position1_feedback,
    motor1_position2_feedback,
    motor1_rpm_feedback,//�������ٶ�0.1rpm
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

    //RCС������
    rc_car_speed = 155,
    rc_car_mileage_high,
    rc_car_mileage_low,
    //������
    Ultrasonic1,
    Ultrasonic2,
    // ��ģ����
    rc_connect_state = 160,//��ģ���ճɹ�
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
    rc_gears_difference,//���˰�����
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
    // RW����
    // С������
    CAN_baud = 195,
    moddbus_485_id,
    moddbus_485_baud,
    control_mode,
    light_control,
    power_control,
    software_reset,
    error_clearance,
    can_reinitialize,//CAN���߳�ʼ��s
    para_save,

    robot_acceleration,
    robot_forward_direction,	//ǰ������
    robot_turning_direction,	//ת�䷽��
    max_linear_speed,
    max_yaw_speed,
    max_angle,
    target_linear_speed,//Ŀ�����ٶȣ�10-3m/s)
    target_yaw_speed,//Ŀ����ٶȣ�10-3rad/s)
    target_angle,
    linear_low,
    linear_middle,
    linear_high,
    angular_low,
    angular_middle,
    angular_high,

    // �������
	motor1_CAN_id = 220,
    motor1_reduction_ratio,//���ٱ�
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
    ro_motor_gap = 340,//������͵ĵ�ַ��ֵ
    rw_motor_gap,//���CANid�ĵ�ַ��ֵ
    torque_cofficient,//���ص�λ��1-3
    undefined343, 
    undefined344,    
    // ��ģ����
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
    rc_encoder_reset = 365,//< �ò���д5����������ͱ���������
    rc_encoder_dir,//< 0:������������Ӽ�;1:�������ֻ�ۼ�
    rc_ratio,//< ���ٱȣ�����0.001
    rc_encoder_accuracy,//< ���������ȣ�����1
    rc_tire_diameter,//< ��ֱ̥������λmm
    rc_speed_shifting,//< RC�ٶ�ƫ��
    rc_angle_shifting,//< RCת��ƫ��
    rc_magnification,//< RC�ٶȱ���
    ir_functioncode,//���⹦���룺1������ɹ���2������ʧ�ܣ�
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


//������ʽ
enum enum_driving_method
{
    front_drive,   //ǰ��
    rear_drive,    //����
    full_drive,   //ȫ��
};

//��س���
enum enum_battery_manufacturer
{
    batterydefault,//Ĭ��
    lishen,//���  
};

//��������
enum enum_motor_type
{
    servo_zlac=0,//< �����ŷ�
    servo_wanze,//< �����ŷ�
    servo_zlacd,//< �����ŷ�һ�϶�����
    servo_plan,//< �����ŷ�
};

//����״̬
enum enum_car_running_state
{
    car_standby,   //����
    car_running,    //����
    car_error,   //����
    car_emergency_stop,   //��ͣ
};

//�����˹�����Ϣ
enum enum_car_error_information
{
    car_normal,             //����
    single_motor_error,     //���������
    multiple_motor_errors,  //��������
    spi_error,              //spi����
};

//�˶�ģʽ
enum enum_sport_mode
{
	position_mode,    //λ��ģʽ
	speed_mode,    // �ٶ�ģʽ
	torque_mode,    // ����ģʽ
};
extern enum enum_sport_mode sport_mode;

// ���ʹ��״̬
enum enum_enable_state
{
	disable_state,   // ʧ��
	enable_state,    // ʹ��
};

// ����ڵ�״̬
enum enum_node_state
{
    node_standby,   		//����
    node_running,   		//����
    node_error,   			//����
    node_disconnect,   	    //����
};

// �����������Ĺ���״̬
enum enum_error_state
{
    error_none = 0x00,  // �޴���
    error_over_value = 0x01,  // ��ѹ
    error_less_value = 0x02,  // Ƿѹ
    error_over_current = 0x04,  // ����
    error_over_load = 0x08,  // ����
    error_overdiff_current = 0x10,  // ��������
    error_overdiff_encoder = 0x20,  // ����������
    error_overdiff_velocity = 0x40,  // �ٶȳ���
    error_ref_value = 0x80,  // �ο���ѹ����
    error_eeprom = 0x100, // EEPROM��д����
    error_hall = 0x200  // ��������
};

// ö�ٶ���С���Ŀ��Ʒ�ʽ
enum enum_control_mode
{
	control_mode_unknown,    // δ֪����ģʽ
	control_mode_remote,    // ��ģ���Ʒ�ʽ
	control_mode_ros,    // ROS����ģʽ
	control_mode_ipc,    	// ��λ������ģʽ
	control_mode_other,     // ��������ģʽ
};

// �����˵ƹ����
enum enum_light_state
{
	light_off,    // ���ƹر�
	light_gleam,    // ������˸
	light_on,    // ���Ƴ���
	light_emergency,    // ��ͣ
};

// �����λ����
enum enum_software_reset
{
	none_reset,    // ����
	once_reset,    // �������
};

// �ֱ�����״̬
enum enum_rc_state
{
	rc_state_failed,  	// δ����
	rc_state_success,  	// ����
};

// CAN������
enum enum_can_baud
{
	can_baud_1000000,  	// 1M
	can_baud_500000,  	// 500K
};

// ���ڲ�����
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

// һЩ���õ���ֵ
enum enum_setting_values
{
	rc_encoder_clear = 5,  	        //��������ͱ���������
	rc_encoder_signed = 0,          //������������Ӽ�
	rc_encoder_unsigned = 1,        //�������ֻ�ۼ�   
    car_software_reset = 0xA5,      //�����λһ��  
    car_error_clearance = 1,        //�������
    car_can_reinit = 5,             //can���³�ʼ��
    car_direct_forward = 1,         //��ת
    car_direct_revert = -1,         //��ת
    power_board_version_old = 0,    //�ɵ�Դ��
    power_board_version_new = 1,    //�µ�Դ��
};



#endif
