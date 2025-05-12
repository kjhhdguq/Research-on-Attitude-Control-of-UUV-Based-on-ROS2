#include "robot_base/robot_base.h"

RobotBase::~RobotBase()
{
    serial_.close();
}

RobotBase::RobotBase(std::string nodeName) : Node(nodeName)
{
    // 加载参数
    std::string port_name = "ttyS3";
    // this->declare_parameter<std::string>("port_name");           //声明及获取串口号参数
    // this->get_parameter_or<std::string>("port_name", port_name, "ttyS3");

    status_publisher_ = this->create_publisher<robot_interfaces::msg::RobotStatus>("robot_status", 10);
    imu_publisher_ = this->create_publisher<robot_interfaces::msg::Imu>("imu", 10);

    cmd_vel_subscription_ = this->create_subscription<robot_interfaces::msg::Command>("robot_cmd", 20, std::bind(&RobotBase::cmd_vel_callback, this, _1));
    mode_vel_subscription_ = this->create_subscription<robot_interfaces::msg::Mode>("robot_mode", 20, std::bind(&RobotBase::mode_vel_callback, this, _1));
    motor_vel_subscription_ = this->create_subscription<robot_interfaces::msg::Motor>("robot_motor", 20, std::bind(&RobotBase::motor_vel_callback, this, _1));
    cali_vel_subscription_ = this->create_subscription<robot_interfaces::msg::Calibration>("robot_cali", 20, std::bind(&RobotBase::cali_callback, this, _1));
    ledf_vel_subscription_ = this->create_subscription<robot_interfaces::msg::StatusControl>("robot_ledf", 20, std::bind(&RobotBase::ledf_callback, this, _1));

    buzzer_service_ = this->create_service<robot_interfaces::srv::RobotBuzzer>("robot_buzzer", std::bind(&RobotBase::buzzer_callback, this, _1, _2));
    led_service_ = this->create_service<robot_interfaces::srv::RobotLed>("robot_led", std::bind(&RobotBase::led_callback, this, _1, _2));
    // left_pid_service_ = this->create_service<robot_interfaces::srv::RobotPID>("robot_left_pid", std::bind(&RobotBase::left_pid_callback, this, _1, _2));
    // right_pid_service_ = this->create_service<robot_interfaces::srv::RobotPID>("robot_right_pid", std::bind(&RobotBase::right_pid_callback, this, _1, _2));

    pid_subscription_ = this->create_subscription<robot_interfaces::msg::PID>("robot_pid", 20, std::bind(&RobotBase::pid_callback, this, _1));

    try
    {
        serial_.setPort("/dev/ttyS3");                                  // 选择要开启的串口号
        serial_.setBaudrate(115200);                                    // 设置波特率
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000); // 超时等待
        serial_.setTimeout(timeOut);
        serial_.open(); // 开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "originbot can not open serial port,Please check the serial port cable! "); // 如果开启串口失败，打印错误信息
    }

    // 如果串口打开，则驱动读取数据的线程
    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "originbot serial port opened"); // 串口开启成功提示

        // 启动一个新线程读取并处理串口数据
        read_data_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&RobotBase::readRawData, this)));
    }

    // timer_100ms_ = this->create_wall_timer(100ms, std::bind(&RobotBase::timer_100ms_callback, this));
    timer_100ms_ = this->create_wall_timer(100ms, std::bind(&RobotBase::timer_100ms_callback, this));
}

void RobotBase::readRawData()
{
    uint8_t rx_data = 0;
    DataFrame frame;

    while (rclcpp::ok())
    {
        // 读取一个字节数据，寻找帧头
        auto len = serial_.read(&rx_data, 1);
        if (len < 1)
            continue;

        // 发现帧头后开始处理数据帧
        if (rx_data == 0x55)
        {
            // 读取完整的数据帧
            serial_.read(&frame.id, 10);
            // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
            //             rx_data, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2],
            //             frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

            // 判断帧尾是否正确
            if (frame.tail != 0xbb)
            {
                // RCLCPP_WARN(this->get_logger(), "Data frame tail error!");
                // printf("Frame raw data[Error]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
                // frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2],
                // frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

                continue;
            }

            frame.header = 0x55;

            // 帧校验
            if (checkDataFrame(frame))
            {
                // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
                //         frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2],
                //         frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

                // 处理帧数据
                processDataFrame(frame);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Data frame check failed!");
            }
        }
    }
}

bool RobotBase::checkDataFrame(DataFrame &frame)
{
    if (((frame.data[0] + frame.data[1] + frame.data[2] +
          frame.data[3] + frame.data[4] + frame.data[5]) &
         0xff) == frame.check)
        return true;
    else
        return false;
}

void RobotBase::processDataFrame(DataFrame &frame)
{
    // 根据数据帧的ID，对应处理帧数据
    switch (frame.id)
    {
    // case FRAME_ID_VELOCITY:
    //     // processVelocityData(frame);
    //     RCLCPP_ERROR(this->get_logger(), "Frame ID Error[1]");
    //     break;
    // case FRAME_ID_ACCELERATION:
    //     processAccelerationData(frame);
    //     RCLCPP_ERROR(this->get_logger(), "Frame ID Error[2]");
    //     break;
    case FRAME_ID_ANGULAR:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[3]");
        processAngularData(frame);
        break;
    case FRAME_ID_EULER:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[4]");
        processEulerData(frame);
        break;
    case FRAME_ID_PIDGyro:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[5]");
        processPidGyroData(frame);
        break;
        // case FRAME_ID_SENSOR:
        //     processSensorData(frame);
        //     RCLCPP_ERROR(this->get_logger(), "Frame ID Error[5]");
        //     break;
        // default:
        //     RCLCPP_ERROR(this->get_logger(), "Frame ID Error[6]");
        //     break;
    }
}

bool RobotBase::buzzer_control(bool on)
{
    DataFrame_8 configFrame;

    // 封装蜂鸣器指令的数据帧
    configFrame.header = 0x55;
    configFrame.id = 0x07;
    configFrame.length = 0x08;
    configFrame.data[0] = 0x00;
    configFrame.data[1] = 0x00;
    configFrame.data[2] = 0xFF;

    if (on)
        configFrame.data[3] = 0xFF;
    else
        configFrame.data[3] = 0x00;

    configFrame.data[4] = 0x00;
    configFrame.data[5] = 0x00;
    configFrame.data[6] = 0x00;
    configFrame.data[7] = 0x00;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + configFrame.data[3] +
                         configFrame.data[4] + configFrame.data[5] + configFrame.data[6] + configFrame.data[7]) &
                        0xff;
    configFrame.tail = 0xbb;

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    return true;
}

bool RobotBase::led_control(bool on)
{
    DataFrame_8 configFrame;

    // 封装控制LED指令的数据帧
    configFrame.header = 0x55;
    configFrame.id = 0x07;
    configFrame.length = 0x08;
    configFrame.data[0] = 0xFF;

    if (on)
        configFrame.data[1] = 0xFF;
    else
        configFrame.data[1] = 0x00;

    configFrame.data[2] = 0x00;
    configFrame.data[3] = 0x00;
    configFrame.data[4] = 0x00;
    configFrame.data[5] = 0x00;
    configFrame.data[6] = 0x00;
    configFrame.data[7] = 0x00;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + configFrame.data[3] +
                         configFrame.data[4] + configFrame.data[5] + configFrame.data[6] + configFrame.data[7]) &
                        0xff;
    configFrame.tail = 0xbb;

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    return true;
}

void RobotBase::buzzer_callback(const std::shared_ptr<robot_interfaces::srv::RobotBuzzer::Request> request,
                                std::shared_ptr<robot_interfaces::srv::RobotBuzzer::Response> response)
{
    robot_status_.buzzer_on = request->on;

    if (buzzer_control(robot_status_.buzzer_on))
    {
        RCLCPP_INFO(this->get_logger(), "Set Buzzer state to %d", robot_status_.buzzer_on);
        response->result = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set Buzzer state error [%d]", robot_status_.buzzer_on);
        response->result = false;
    }
}

void RobotBase::led_callback(const std::shared_ptr<robot_interfaces::srv::RobotLed::Request> request,
                             std::shared_ptr<robot_interfaces::srv::RobotLed::Response> response)
{
    robot_status_.led_on = request->on;
    RCLCPP_INFO(this->get_logger(), "led_service opened");
    if (led_control(robot_status_.led_on))
    {
        RCLCPP_INFO(this->get_logger(), "Set Led state to %d", robot_status_.led_on);
        response->result = true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Set Led state error [%d]", robot_status_.led_on);
        response->result = false;
    }
}

void RobotBase::ledf_callback(const robot_interfaces::msg::StatusControl &ledf)
{
    DataFrame_8 configFrame;

    // 封装控制LED指令的数据帧
    configFrame.header = 0x55;
    configFrame.id = 0x07;
    configFrame.length = 0x08;
    configFrame.data[0] = 0x00;
    configFrame.data[1] = 0x00;
    configFrame.data[2] = 0x00;
    configFrame.data[3] = 0x00;
    configFrame.data[4] = 0x00;
    configFrame.data[5] = 0x00;
    configFrame.data[6] = 0xFF;
    if (ledf.result)
        configFrame.data[7] = 0xFF;
    else
        configFrame.data[7] = 0x00;

    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + configFrame.data[3] +
                         configFrame.data[4] + configFrame.data[5] + configFrame.data[6] + configFrame.data[7]) &
                        0xff;
    configFrame.tail = 0xbb;

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }
    if (ledf.result == 1)
    {
        RCLCPP_INFO(this->get_logger(), "Ledf_ON");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Ledf_OFF");
    }
}

void RobotBase::cali_callback(const robot_interfaces::msg::Calibration &cali)
{
    DataFrame configFrame;

    // 封装控制LED指令的数据帧
    configFrame.header = 0x55;
    configFrame.id = 0x07;
    configFrame.length = 0x08;
    configFrame.data[0] = 0x00;
    configFrame.data[1] = 0x00;
    configFrame.data[2] = 0x00;
    configFrame.data[3] = 0x00;
    configFrame.data[4] = 0xFF;
    configFrame.data[6] = 0x00;
    configFrame.data[7] = 0x00;
    if (cali.result)
        configFrame.data[5] = 0xFF;
    else
        configFrame.data[5] = 0x00;

    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + configFrame.data[3] +
                         configFrame.data[4] + configFrame.data[5] + configFrame.data[6] + configFrame.data[7]) &
                        0xff;
    configFrame.tail = 0xbb;

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    RCLCPP_INFO(this->get_logger(), "Set JY901 Calibration");
}

void RobotBase::pid_callback(const robot_interfaces::msg::PID &pid)
{
    DataFrame pidFrame;

    short id = pid.id;
    pidFrame.header = 0x55;
    pidFrame.id = 0x0A + id;
    pidFrame.length = 0x06;
    if (id == 0)
    {
        short pid_flag = pid.pid_flag;
        pidFrame.data[0] = 0xFF;
        pidFrame.data[1] = 0xFF;
        pidFrame.data[2] = 0xFF;
        pidFrame.data[3] = 0xFF;
        pidFrame.data[4] = 0xFF;
        pidFrame.data[5] = pid_flag;
    }
    else if (id <= 6 && id > 0)
    {
        uint16_t p = (uint16_t)(pid.p * 10000);
        uint16_t i = (uint16_t)(pid.i * 10000);
        uint16_t d = (uint16_t)(pid.d * 10000);
        pidFrame.data[0] = p & 0xFF;
        pidFrame.data[1] = (p >> 8) & 0xFF;
        pidFrame.data[2] = i & 0xFF;
        pidFrame.data[3] = (i >> 8) & 0xFF;
        pidFrame.data[4] = d & 0xFF;
        pidFrame.data[5] = (d >> 8) & 0xFF;
    }
    else
    {
        short pid_interval = pid.pid_interval;
        pidFrame.data[0] = 0xFF;
        pidFrame.data[1] = 0xFF;
        pidFrame.data[2] = 0xFF;
        pidFrame.data[3] = 0xFF;
        pidFrame.data[4] = pid_interval & 0xFF;
        pidFrame.data[5] = (pid_interval >> 8) & 0xFF;
    }
    pidFrame.check = (pidFrame.data[0] + pidFrame.data[1] + pidFrame.data[2] +
                      pidFrame.data[3] + pidFrame.data[4] + pidFrame.data[5]) &
                     0xff;
    pidFrame.tail = 0xbb;

    try
    {
        serial_.write(&pidFrame.header, sizeof(pidFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    if (id == 0)
    {
        if (pid.pid_flag == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Begin to Setup pid");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "End to Setup pid");
        }
    }
    else if (id <= 6 && id > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Set pid%d parameters to [%0.4f %0.4f %0.4f]", pid.id, pid.p, pid.i, pid.d);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Set pid_interval parameters to %d", pid.pid_interval);
    }
}

bool Led_my = 0;

void RobotBase::timer_100ms_callback()
{
    // Led_my = !Led_my;
    // RobotBase::led_control(Led_my);

    // 发布机器人的状态信息
    robot_interfaces::msg::RobotStatus status_msg;

    status_msg.buzzer_on = robot_status_.buzzer_on;
    status_msg.led_on = robot_status_.led_on;

    status_publisher_->publish(status_msg);
}

void RobotBase::cmd_vel_callback(const robot_interfaces::msg::Command &cmd)
{
    DataFrame_12 CmdFrame;

    // rightleft    --yaw
    // goback       --pitch
    //
    short goback = (short)(cmd.goback);
    short rightleft = (short)(cmd.rightleft);
    short updown = (short)(cmd.updown);
    short yaw = (short)(cmd.yaw);
    short pitch = (short)(cmd.pitch);
    short roll = (short)(cmd.roll);
    // 封装运动指令的数据帧
    CmdFrame.header = 0x55;
    CmdFrame.id = 0x09;
    CmdFrame.length = 0x0C;
    CmdFrame.data[0] = updown & 0xFF;
    CmdFrame.data[1] = (updown >> 8) & 0xFF;
    CmdFrame.data[2] = yaw & 0xFF;
    CmdFrame.data[3] = (yaw >> 8) & 0xFF;
    CmdFrame.data[4] = pitch & 0xFF;
    CmdFrame.data[5] = (pitch >> 8) & 0xFF;
    CmdFrame.data[6] = roll & 0xFF;
    CmdFrame.data[7] = (roll >> 8) & 0xFF;
    CmdFrame.data[8] = goback & 0xFF;
    CmdFrame.data[9] = (goback >> 8) & 0xFF;
    CmdFrame.data[10] = rightleft & 0xFF;
    CmdFrame.data[11] = (rightleft >> 8) & 0xFF;
    CmdFrame.check = (CmdFrame.data[0] + CmdFrame.data[1] + CmdFrame.data[2] + CmdFrame.data[3] +
                      CmdFrame.data[4] + CmdFrame.data[5] + CmdFrame.data[6] + CmdFrame.data[7] +
                      CmdFrame.data[8] + CmdFrame.data[9] + CmdFrame.data[10] + CmdFrame.data[11]) &
                     0xff;
    CmdFrame.tail = 0xbb;

    try
    {
        serial_.write(&CmdFrame.header, sizeof(CmdFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }
    
    RCLCPP_INFO(this->get_logger(), "前后goback: %d", cmd.goback);
    RCLCPP_INFO(this->get_logger(), "横移rightleft: %d", cmd.rightleft);
    RCLCPP_INFO(this->get_logger(), "升降updown: %d", cmd.updown);
    RCLCPP_INFO(this->get_logger(), "偏航yaw: %d", cmd.yaw);
    RCLCPP_INFO(this->get_logger(), "俯仰pitch: %d", cmd.pitch);
    RCLCPP_INFO(this->get_logger(), "滚转roll: %d", cmd.roll);
}

void RobotBase::mode_vel_callback(const robot_interfaces::msg::Mode &mode)
{
    DataFrame ModeFrame;

    bool on = mode.on;
    bool result = mode.result;

    // 封装运动指令的数据帧
    ModeFrame.header = 0x55;
    ModeFrame.id = 0x08;
    ModeFrame.length = 0x06;
    ModeFrame.data[0] = 0xFF;
    ModeFrame.data[1] = 0xFF;
    ModeFrame.data[2] = 0xFF;
    ModeFrame.data[3] = 0xFF;
    ModeFrame.data[4] = on;
    ModeFrame.data[5] = result;
    ModeFrame.check = (ModeFrame.data[0] + ModeFrame.data[1] + ModeFrame.data[2] +
                       ModeFrame.data[3] + ModeFrame.data[4] + ModeFrame.data[5]) &
                      0xff;
    ModeFrame.tail = 0xbb;

    try
    {
        serial_.write(&ModeFrame.header, sizeof(ModeFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    RCLCPP_INFO(this->get_logger(), "控制方式为 %d", mode.result);
}

void RobotBase::motor_vel_callback(const robot_interfaces::msg::Motor &motor)
{
    DataFrame MotorFrame;

    int8_t id = motor.id;
    int16_t speed = motor.speed;

    // 封装运动指令的数据帧
    MotorFrame.header = 0x55;
    MotorFrame.id = 0x11;
    MotorFrame.length = 0x06;
    MotorFrame.data[0] = 0xFF;
    MotorFrame.data[1] = 0xFF;
    MotorFrame.data[2] = 0xFF;
    MotorFrame.data[3] = id;
    MotorFrame.data[4] = speed & 0xFF;
    ;
    MotorFrame.data[5] = (speed >> 8) & 0xFF;
    MotorFrame.check = (MotorFrame.data[0] + MotorFrame.data[1] + MotorFrame.data[2] +
                        MotorFrame.data[3] + MotorFrame.data[4] + MotorFrame.data[5]) &
                       0xff;
    MotorFrame.tail = 0xbb;

    try
    {
        serial_.write(&MotorFrame.header, sizeof(MotorFrame)); // 向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // 如果发送数据失败,打印错误信息
    }

    if (motor.id == 1 || motor.id == 2 || motor.id == 6 || motor.id == 7)
    {
        RCLCPP_INFO(this->get_logger(), "电机 %d 的速度为 %d", motor.id, -motor.speed);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "电机 %d 的速度为 %d", motor.id, motor.speed);
    }
}

void RobotBase::imu_publisher()
{
    // RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");

    // 封装IMU的话题消息
    // auto imu_msg = sensor_msgs::msg::Imu();

    // imu_msg.header.frame_id = "imu_link";
    // imu_msg.header.stamp = this->get_clock()->now();

    // imu_msg.linear_acceleration.x = imu_data_.acceleration_x;
    // imu_msg.linear_acceleration.y = imu_data_.acceleration_y;
    // imu_msg.linear_acceleration.z = imu_data_.acceleration_z;

    // imu_msg.angular_velocity.x = imu_data_.angular_x;
    // imu_msg.angular_velocity.y = imu_data_.angular_y;
    // imu_msg.angular_velocity.z = imu_data_.angular_z;

    // tf2::Quaternion q;
    // q.setRPY(imu_data_.roll, imu_data_.pitch, imu_data_.yaw);

    // imu_msg.orientation.x = q[0];
    // imu_msg.orientation.y = q[1];
    // imu_msg.orientation.z = q[2];
    // imu_msg.orientation.w = q[3];

    // imu_msg.linear_acceleration_covariance = {0.04, 0.00, 0.00, 0.00, 0.04, 0.00, 0.00, 0.00, 0.04};

    // imu_msg.angular_velocity_covariance = {0.02, 0.00, 0.00, 0.00, 0.02, 0.00, 0.00, 0.00, 0.02};

    // imu_msg.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025, 0.0000, 0.0000, 0.0000, 0.0025};

    // // 发布IMU话题
    // imu_publisher_->publish(imu_msg);

    // RCLCPP_INFO(this->get_logger(), "data: %f %f %f", imu_data_.roll, imu_data_.pitch, imu_data_.yaw);
}

void RobotBase::processAccelerationData(DataFrame &frame)
{
    // RCLCPP_INFO(this->get_logger(), "Process acceleration data");

    imu_data_.acceleration_x = imu_conversion(frame.data[1], frame.data[0]) / 32768 * 16 * 9.8;
    imu_data_.acceleration_y = imu_conversion(frame.data[3], frame.data[2]) / 32768 * 16 * 9.8;
    imu_data_.acceleration_z = imu_conversion(frame.data[5], frame.data[4]) / 32768 * 16 * 9.8;
}

void RobotBase::processAngularData(DataFrame &frame)
{
    // RCLCPP_INFO(this->get_logger(), "Process angular data");

    imu_data_.angular_x = imu_conversion(frame.data[1], frame.data[0]) / 32768.0 * degToRad(2000);
    imu_data_.angular_y = imu_conversion(frame.data[3], frame.data[2]) / 32768.0 * degToRad(2000);
    imu_data_.angular_z = imu_conversion(frame.data[5], frame.data[4]) / 32768.0 * degToRad(2000);

    RCLCPP_INFO(this->get_logger(), "data: %f %f %f", imu_data_.angular_x, imu_data_.angular_y, imu_data_.angular_z);
}

void RobotBase::processEulerData(DataFrame &frame)
{
    // RCLCPP_INFO(this->get_logger(), "Process euler data");

    imu_data_.roll = imu_conversion(frame.data[1], frame.data[0]) / 32768.0 * 180;
    imu_data_.pitch = imu_conversion(frame.data[3], frame.data[2]) / 32768.0 * 180;
    imu_data_.yaw = imu_conversion(frame.data[5], frame.data[4]) / 32768.0 * 180;

    RCLCPP_INFO(this->get_logger(), "data: %f %f %f", imu_data_.roll, imu_data_.pitch, imu_data_.yaw);

    // imu_publisher();
}

void RobotBase::processSensorData(DataFrame &frame)
{
    robot_status_.battery_voltage = (float)frame.data[0] + ((float)frame.data[1] / 100.0);

    // RCLCPP_INFO(this->get_logger(), "Battery Voltage: %0.2f", (float)frame.data[0] + ((float)frame.data[1]/100.0));
}

void RobotBase::processPidGyroData(DataFrame &frame)
{
    double PidRateXOut;
    double PidRateYOut;
    double PidRateZOut;
    PidRateXOut = imu_conversion(frame.data[1], frame.data[0]);
    PidRateYOut = imu_conversion(frame.data[3], frame.data[2]);
    PidRateZOut = imu_conversion(frame.data[5], frame.data[4]);

    RCLCPP_INFO(this->get_logger(), "data: %f %f %f", PidRateXOut, PidRateYOut, PidRateZOut);
}

double RobotBase::degToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

double RobotBase::imu_conversion(uint8_t data_high, uint8_t data_low)
{
    short transition_16;

    transition_16 = 0;
    transition_16 |= data_high << 8;
    transition_16 |= data_low;

    return transition_16;
}

void sigintHandler(int sig)
{
    sig = sig;

    printf("OriginBot shutdown...\n");

    serial::Serial serial;
    serial.setPort("/dev/ttyS3");                                   // 选择要开启的串口号
    serial.setBaudrate(921600);                                     // 设置波特率
    serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000); // 超时等待
    serial.setTimeout(timeOut);
    serial.open(); // 开启串口

    // 如果串口打开，则驱动读取数据的线程
    if (serial.isOpen())
    {
        // 程序退出时自动停车
        DataFrame cmdFrame;
        cmdFrame.data[0] = 0x00;
        cmdFrame.data[1] = 0x00;
        cmdFrame.data[2] = 0x00;
        cmdFrame.data[3] = 0x00;
        cmdFrame.data[4] = 0x00;
        cmdFrame.data[5] = 0x00;
        cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] +
                          cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) &
                         0xff;

        // 封装速度命令的数据帧
        cmdFrame.header = 0x55;
        cmdFrame.id = 0x01;
        cmdFrame.length = 0x06;
        cmdFrame.tail = 0xbb;

        try
        {
            serial.write(&cmdFrame.header, sizeof(cmdFrame)); // 向串口发数据
            printf("Execute auto stop\n");
        }
        catch (serial::IOException &e)
        {
            printf("Unable to send data through serial port\n"); // 如果发送数据失败,打印错误信息
        }
    }

    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();
}

// ROS2节点主入口main函数
int main(int argc, char *argv[])
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    signal(SIGINT, sigintHandler);
    // 创建ROS2节点对象并进行初始化
    // rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::spin(std::make_shared<RobotBase>("robot_base"));
    // 关闭ROS2 C++接口
    rclcpp::shutdown();

    return 0;
}
