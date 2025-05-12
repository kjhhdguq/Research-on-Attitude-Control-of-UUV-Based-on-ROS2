#include "robot_teleop/robot_teleop.h"

auto led_request = std::make_shared<robot_interfaces::srv::RobotLed::Request>();
auto buzzer_request = std::make_shared<robot_interfaces::srv::RobotBuzzer::Request>();

int pitch = 0;
int yaw = 0;
int updown = 0;
int roll = 0;
int goback = 0;
int rightleft = 0;

// - - + + + - - +
int16_t motor[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

bool key_motor_flag = 0;
bool enter_motor_flag = 0;

// init启动
RobotTeleop::RobotTeleop(std::string nodeName) : Node(nodeName)
{
    RCLCPP_INFO(this->get_logger(), "Starting up RoBot telop keyboard controller");

    tcgetattr(kfd, &initial_settings);
    new_settings = initial_settings;
    // 使用标准输入模式 | 显示输入字符
    new_settings.c_lflag &= ~(ICANON | ECHO);
    // VEOL: 附加的end of life字符
    new_settings.c_cc[VEOL] = 1;
    // VEOF: end of life字符
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);

    led_client = this->create_client<robot_interfaces::srv::RobotLed>("robot_led");
    buzzer_client = this->create_client<robot_interfaces::srv::RobotBuzzer>("robot_buzzer");
    cmd_publisher_ = this->create_publisher<robot_interfaces::msg::Command>("robot_cmd", 10);
    mode_publisher_ = this->create_publisher<robot_interfaces::msg::Mode>("robot_mode", 10);
    motor_publisher_ = this->create_publisher<robot_interfaces::msg::Motor>("robot_motor", 10);
    cali_publisher_ = this->create_publisher<robot_interfaces::msg::Calibration>("robot_cali", 10);
    ledf_publisher_ = this->create_publisher<robot_interfaces::msg::StatusControl>("robot_ledf", 10);

    while (!led_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!buzzer_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            break;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    led_request->on = 0;
    buzzer_request->on = 0;
    auto led_result = led_client->async_send_request(led_request);
    auto buzzer_result = buzzer_client->async_send_request(buzzer_request);

    // 初始化控制角度
    motion_Cmd.goback = 0;
    motion_Cmd.rightleft = 0;
    motion_Cmd.updown = 0;
    motion_Cmd.yaw = 0;
    motion_Cmd.pitch = 0;
    motion_Cmd.roll = 0;

    // 初始化控制模式
    motion_mode.on = 1;
    motion_mode.result = 1;

    // 初始化是否校准
    imu_calib.on = 1;
    imu_calib.result = 0;

    // 初始化是否开启照明
    ledf_status.on = 1;
    ledf_status.result = 0;

    motion_motor.id = 0;
    motion_motor.speed = 0;

    timer_100ms_ = this->create_wall_timer(100ms, std::bind(&RobotTeleop::timer_100ms_callback, this));

    showMenu();
    teleopKeyboardLoop();
}

void RobotTeleop::SendLedRequest(void)
{
    led_request->on = !(led_request->on);
    auto led_result = led_client->async_send_request(led_request);
    if (led_request->on == 1)
    {
        RCLCPP_INFO(this->get_logger(), "The Led is opened!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The Led is closed!");
    }
}

void RobotTeleop::SendBuzzerRequest(void)
{
    buzzer_request->on = !(buzzer_request->on);
    auto buzzer_result = buzzer_client->async_send_request(buzzer_request);
    if (buzzer_request->on == 1)
    {
        RCLCPP_INFO(this->get_logger(), "The Buzzer is opened!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The Buzzer is closed!");
    }
}

void RobotTeleop::SendMotionMode(void)
{
    motion_mode.result = !(motion_mode.result);
    // auto buzzer_result = buzzer_client->async_send_request(buzzer_request);
    mode_publisher_->publish(motion_mode);
    if (motion_mode.result == 1)
    {
        RCLCPP_INFO(this->get_logger(), "The MotionMode is closed!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The MotionMode is open!");
    }
}

void RobotTeleop::SendLedfStatus(void)
{
    ledf_status.result = !(ledf_status.result);
    ledf_publisher_->publish(ledf_status);
    if (ledf_status.result == 1)
    {
        RCLCPP_INFO(this->get_logger(), "The Ledf is opened!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The Ledf is closed!");
    }
}

void RobotTeleop::SendJY901Cali(void)
{
    imu_calib.result = 1;
    cali_publisher_->publish(imu_calib);
    if (imu_calib.result == 1)
    {
        RCLCPP_INFO(this->get_logger(), "The Calibration of Jy901 start!");
        imu_calib.result = 0;
    }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "The Calibration of Jy901 start!");
    // }
}

void RobotTeleop::stopRobot()
{
}

void RobotTeleop::timer_100ms_callback()
{
}

void RobotTeleop::showMenu()
{
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "|    q   w   e   |   left-roll       forward--pitch    right-roll     |" << std::endl;
    std::cout << "|    a   s   d   |   left-yaw        backward-pitch    right-yaw      |" << std::endl;
    std::cout << "|        p       |                        rise                        |" << std::endl;
    std::cout << "|        l       |                        down                        |" << std::endl;
    std::cout << "|                                                                     |" << std::endl;
    std::cout << "|        ↑       |                      forward                       |" << std::endl;
    std::cout << "|    ←   ↓   →   |     left             backward         right        |" << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;

    std::cout << "M   : open the led                                                    |" << std::endl;
    std::cout << "N   : open the Buzzer                                                 |" << std::endl;
    std::cout << "K   : open the ledf                                                   |" << std::endl;
    std::cout << "F   : Control Mode                                                    |" << std::endl;
    std::cout << "J   : Cali the JY901                                                  |" << std::endl;

    std::cout << std::endl;
    std::cout << "press h to Menu and ctrl+c to quit" << std::endl;
}

void RobotTeleop::teleopKeyboardLoop()
{
    char key_old;

    int fd_kb;
    struct input_event event_kb;

    fd_kb = open("/dev/input/event1", O_RDONLY); // 键盘输入

    if (fd_kb <= 0)
    {
        perror("open device error\n");
    }

    while (rclcpp::ok())
    {
        read(fd_kb, &event_kb, sizeof(event_kb));
        if (event_kb.type == EV_KEY)
        {
            if (event_kb.value == 1) // 1表示按下，0表示释放
            {
                switch (event_kb.code)
                {
                case KEY_W:
                    if (enter_motor_flag == 1)
                    {
                        if (motion_motor.id == 1 || motion_motor.id == 2 || motion_motor.id == 6 || motion_motor.id == 7)
                        {
                            motor[motion_motor.id]--;
                            if (motor[motion_motor.id] <= -90)
                                motor[motion_motor.id] = -90;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度增加为 %d", motion_motor.id, -motion_motor.speed);
                        }
                        else
                        {
                            motor[motion_motor.id]++;
                            if (motor[motion_motor.id] >= 90)
                                motor[motion_motor.id] = 90;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度增加为 %d", motion_motor.id, motion_motor.speed);
                        }
                    }

                    else
                    {
                        motion_Cmd.roll = 0;
                        // motion_Cmd.yaw = 0;
                        pitch = pitch + 1;
                        if (pitch >= 100)
                        {
                            pitch = 100;
                        }
                        motion_Cmd.pitch = pitch;
                        cmd_publisher_->publish(motion_Cmd);
                        RCLCPP_INFO(this->get_logger(), "前俯--速度为 %d", pitch);
                    }
                    break;

                case KEY_S:
                    if (enter_motor_flag == 1)
                    {
                        if (motion_motor.id == 1 || motion_motor.id == 2 || motion_motor.id == 6 || motion_motor.id == 7)
                        {
                            motor[motion_motor.id]++;
                            if (motor[motion_motor.id] >= 0)
                                motor[motion_motor.id] = 0;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度减小为 %d", motion_motor.id, -motion_motor.speed);
                        }
                        else
                        {
                            motor[motion_motor.id]--;
                            if (motor[motion_motor.id] <= 0)
                                motor[motion_motor.id] = 0;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度减小为 %d", motion_motor.id, motion_motor.speed);
                        }
                    }

                    else
                    {
                        motion_Cmd.roll = 0;
                        // motion_Cmd.yaw = 0;
                        pitch = pitch - 1;
                        if (pitch <= -100)
                        {
                            pitch = -100;
                        }
                        motion_Cmd.pitch = pitch;
                        cmd_publisher_->publish(motion_Cmd);
                        RCLCPP_INFO(this->get_logger(), "后仰--速度为 %d", pitch);
                    }
                    break;

                case KEY_A:
                    motion_Cmd.roll = 0;
                    motion_Cmd.pitch = 0;
                    yaw = yaw + 1;
                    if (yaw >= 90)
                    {
                        yaw = 90;
                    }
                    motion_Cmd.yaw = yaw;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左转--速度为 %d", yaw);
                    break;

                case KEY_D:
                    motion_Cmd.roll = 0;
                    motion_Cmd.pitch = 0;
                    yaw = yaw - 1;
                    if (yaw <= -90)
                    {
                        yaw = -90;
                    }
                    motion_Cmd.yaw = yaw;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右转--速度为 %d", yaw);
                    break;

                case KEY_Q:
                    motion_Cmd.pitch = 0;
                    // motion_Cmd.yaw = 0;
                    roll = roll - 20;
                    if (roll <= -720)
                    {
                        roll = -720;
                    }
                    motion_Cmd.roll = roll;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左倾--速度为 %d", roll);
                    break;

                case KEY_E:
                    motion_Cmd.pitch = 0;
                    // motion_Cmd.yaw = 0;
                    roll = roll + 20;
                    if (roll >= 720)
                    {
                        roll = 720;
                    }
                    motion_Cmd.roll = roll;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右倾--速度为 %d", roll);
                    break;

                case KEY_LEFT:
                    motion_Cmd.goback = 0;
                    rightleft = rightleft + 0;
                    if (rightleft >= 300)
                    {
                        rightleft = 300;
                    }
                    motion_Cmd.rightleft = rightleft;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左移--速度为 %d", rightleft);
                    break;

                case KEY_RIGHT:
                    motion_Cmd.goback = 0;
                    rightleft = rightleft - 10;
                    if (rightleft <= -300)
                    {
                        rightleft = -300;
                    }
                    motion_Cmd.rightleft = rightleft;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右移--速度为 %d", rightleft);
                    break;

                case KEY_UP:
                    motion_Cmd.rightleft = 0;
                    goback = goback + 10;
                    if (goback >= 300)
                    {
                        goback = 300;
                    }
                    motion_Cmd.goback = goback;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "前移--速度为 %d", goback);
                    break;

                case KEY_DOWN:
                    motion_Cmd.rightleft = 0;
                    goback = goback - 10;
                    if (goback <= -300)
                    {
                        goback = -300;
                    }
                    motion_Cmd.goback = goback;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "后移--速度为 %d", goback);
                    break;

                case KEY_P:
                    motion_Cmd.updown = motion_Cmd.updown + 1;
                    if (motion_Cmd.updown >= 250)
                    {
                        motion_Cmd.updown = 250;
                    }
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "上浮--程度为 %d", motion_Cmd.updown);
                    break;

                case KEY_L:
                    motion_Cmd.updown = motion_Cmd.updown - 1;
                    if (motion_Cmd.updown <= -250)
                    {
                        motion_Cmd.updown = -250;
                    }
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "下潜--程度为 %d", motion_Cmd.updown);
                    break;

                case KEY_M:
                    SendLedRequest();
                    break;

                case KEY_N:
                    SendBuzzerRequest();
                    break;

                case KEY_K:
                    SendLedfStatus();
                    break;

                case KEY_F:
                    SendMotionMode();
                    break;

                case KEY_J:
                    goback = motion_Cmd.goback = 0;
                    rightleft = motion_Cmd.rightleft = 0;
                    updown = motion_Cmd.updown = 0;
                    yaw = motion_Cmd.yaw = 0;
                    pitch = motion_Cmd.pitch = 0;
                    roll = motion_Cmd.roll = 0;
                    SendJY901Cali();
                    // cmd_publisher_->publish(motion_Cmd);
                    break;

                case KEY_ENTER:
                    if (key_motor_flag == 1 && enter_motor_flag == 0)
                    {
                        enter_motor_flag = 1;
                        RCLCPP_INFO(this->get_logger(), "------已进入电机调节------");
                    }
                    else if (key_motor_flag == 0 && enter_motor_flag == 1)
                    {
                        enter_motor_flag = 0;
                        RCLCPP_INFO(this->get_logger(), "------已退出电机调节------");
                    }
                    break;

                case KEY_SPACE:
                    if (enter_motor_flag == 1)
                    {
                        motor[motion_motor.id] = 0;
                        motion_motor.speed = motor[motion_motor.id];
                        motor_publisher_->publish(motion_motor);
                        RCLCPP_INFO(this->get_logger(), "------电机 %d 已停止------", motion_motor.id);
                    }
                    break;
                }
                key_old = event_kb.code;
            }

            if (event_kb.value == 2) // 2表示长按
            {
                switch (event_kb.code)
                {
                case KEY_W:
                    if (enter_motor_flag == 1)
                    {
                        if (motion_motor.id == 1 || motion_motor.id == 2 || motion_motor.id == 6 || motion_motor.id == 7)
                        {
                            motor[motion_motor.id]--;
                            if (motor[motion_motor.id] <= -90)
                                motor[motion_motor.id] = -90;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度增加为 %d", motion_motor.id, -motion_motor.speed);
                        }
                        else
                        {
                            motor[motion_motor.id]++;
                            if (motor[motion_motor.id] >= 90)
                                motor[motion_motor.id] = 90;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度增加为 %d", motion_motor.id, motion_motor.speed);
                        }
                    }

                    else
                    {
                        motion_Cmd.roll = 0;
                        // motion_Cmd.yaw = 0;
                        pitch = pitch + 5;
                        if (pitch >= 200)
                        {
                            pitch = 200;
                        }
                        motion_Cmd.pitch = pitch;
                        cmd_publisher_->publish(motion_Cmd);
                        RCLCPP_INFO(this->get_logger(), "前俯--速度为 %d", pitch);
                    }
                    break;

                case KEY_S:
                    if (enter_motor_flag == 1)
                    {
                        if (motion_motor.id == 1 || motion_motor.id == 2 || motion_motor.id == 6 || motion_motor.id == 7)
                        {
                            motor[motion_motor.id]++;
                            if (motor[motion_motor.id] >= 0)
                                motor[motion_motor.id] = 0;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度减小为 %d", motion_motor.id, -motion_motor.speed);
                        }
                        else
                        {
                            motor[motion_motor.id]--;
                            if (motor[motion_motor.id] <= 0)
                                motor[motion_motor.id] = 0;
                            motion_motor.speed = motor[motion_motor.id];
                            motor_publisher_->publish(motion_motor);
                            RCLCPP_INFO(this->get_logger(), "电机 %d 的速度减小为 %d", motion_motor.id, motion_motor.speed);
                        }
                    }

                    else
                    {
                        motion_Cmd.roll = 0;
                        // motion_Cmd.yaw = 0;
                        pitch = pitch - 5;
                        if (pitch <= -200)
                        {
                            pitch = -200;
                        }
                        motion_Cmd.pitch = pitch;
                        cmd_publisher_->publish(motion_Cmd);
                        RCLCPP_INFO(this->get_logger(), "后仰--速度为 %d", pitch);
                    }
                    break;

                case KEY_A:
                    motion_Cmd.roll = 0;
                    motion_Cmd.pitch = 0;
                    yaw = yaw + 1;
                    if (yaw >= 90)
                    {
                        yaw = 90;
                    }
                    motion_Cmd.yaw = yaw;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左转--速度为 %d", yaw);
                    break;

                case KEY_D:
                    motion_Cmd.roll = 0;
                    motion_Cmd.pitch = 0;
                    yaw = yaw - 1;
                    if (yaw <= -90)
                    {
                        yaw = -90;
                    }
                    motion_Cmd.yaw = yaw;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右转--速度为 %d", yaw);
                    break;

                case KEY_Q:
                    motion_Cmd.pitch = 0;
                    // motion_Cmd.yaw = 0;
                    roll = roll - 20;
                    if (roll <= -720)
                    {
                        roll = -720;
                    }
                    motion_Cmd.roll = roll;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左倾--速度为 %d", roll);
                    break;

                case KEY_E:
                    motion_Cmd.pitch = 0;
                    // motion_Cmd.yaw = 0;
                    roll = roll + 20;
                    if (roll >= 720)
                    {
                        roll = 720;
                    }
                    motion_Cmd.roll = roll;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右倾--速度为 %d", roll);
                    break;

                case KEY_LEFT:
                    motion_Cmd.goback = 0;
                    rightleft = rightleft + 10;
                    if (rightleft >= 300)
                    {
                        rightleft = 300;
                    }
                    motion_Cmd.rightleft = rightleft;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "左移--速度为 %d", rightleft);
                    break;

                case KEY_RIGHT:
                    motion_Cmd.goback = 0;
                    rightleft = rightleft - 10;
                    if (rightleft <= -300)
                    {
                        rightleft = -300;
                    }
                    motion_Cmd.rightleft = rightleft;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "右移--速度为 %d", rightleft);
                    break;

                case KEY_UP:
                    motion_Cmd.rightleft = 0;
                    goback = goback + 10;
                    if (goback >= 300)
                    {
                        goback = 300;
                    }
                    motion_Cmd.goback = goback;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "前移--速度为 %d", goback);
                    break;

                case KEY_DOWN:
                    motion_Cmd.rightleft = 0;
                    goback = goback - 10;
                    if (goback <= -300)
                    {
                        goback = -300;
                    }
                    motion_Cmd.goback = goback;
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "后移--速度为 %d", goback);
                    break;

                case KEY_P:
                    motion_Cmd.updown = motion_Cmd.updown + 1;
                    if (motion_Cmd.updown >= 250)
                    {
                        motion_Cmd.updown = 250;
                    }
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "上浮--程度为 %d", motion_Cmd.updown);
                    break;

                case KEY_L:
                    motion_Cmd.updown = motion_Cmd.updown - 1;
                    if (motion_Cmd.updown <= -250)
                    {
                        motion_Cmd.updown = -250;
                    }
                    cmd_publisher_->publish(motion_Cmd);
                    RCLCPP_INFO(this->get_logger(), "下潜--程度为 %d", motion_Cmd.updown);
                    break;

                default:
                    break;
                }
                key_old = event_kb.code;
            }
            if (event_kb.value == 0) // 1表示按下，0表示释放
            {
                if (key_old == KEY_W || key_old == KEY_S || key_old == KEY_UP || key_old == KEY_DOWN ||
                    key_old == KEY_LEFT || key_old == KEY_RIGHT || key_old == KEY_Q || key_old == KEY_E)
                {
                    if (key_motor_flag == 0 && enter_motor_flag == 0)
                    {
                        // 航向角目标角度不置零，而俯仰和滚转置零
                        pitch = roll = goback = rightleft = 0;
                        motion_Cmd.pitch = 0;
                        motion_Cmd.roll = 0;
                        motion_Cmd.goback = 0;
                        motion_Cmd.rightleft = 0;
                        cmd_publisher_->publish(motion_Cmd);
                        RCLCPP_INFO(this->get_logger(), "------停止------");
                    }
                }
                switch (key_old)
                {
                case KEY_0:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 0;
                        key_motor_flag = 0;
                    }
                    if (enter_motor_flag == 0)
                    {
                        key_motor_flag = 1;
                    }
                    break;

                case KEY_1:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 1;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 1 ------");
                    }
                    break;

                case KEY_2:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 2;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 2 ------");
                    }
                    break;

                case KEY_3:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 3;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 3 ------");
                    }
                    break;

                case KEY_4:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 4;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 4 ------");
                    }
                    break;

                case KEY_5:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 5;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 5 ------");
                    }
                    break;

                case KEY_6:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 6;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 6 ------");
                    }
                    break;

                case KEY_7:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 7;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 7 ------");
                    }
                    break;

                case KEY_8:
                    if (enter_motor_flag == 1)
                    {
                        motion_motor.id = 8;
                        RCLCPP_INFO(this->get_logger(), "------可以调节电机 8 ------");
                    }
                    break;

                default:
                    break;
                }
            }
        }
    }

    std::map<char, std::tuple<int, int, std::string>> keymap = {
        {KEYCODE_W, std::make_tuple(1, 0, "Move forward")},
        {KEYCODE_S, std::make_tuple(-1, 0, "Move backward")},
        {KEYCODE_A, std::make_tuple(0, 1, "Left rotate")},
        {KEYCODE_D, std::make_tuple(0, -1, "Right rotate")},
        {KEYCODE_C, std::make_tuple(-1, 1, "Left backward turn")},
        {KEYCODE_Z, std::make_tuple(-1, -1, "Right backward turn")},
        {KEYCODE_Q, std::make_tuple(1, 1, "Right forward turn")},
        {KEYCODE_E, std::make_tuple(1, -1, "Left forward turn")},

        {KEYCODE_H, std::make_tuple(0, 0, "Show Menu")},

        {KEYCODE_I, std::make_tuple(0, 0, "Increase linear speed")},
        {KEYCODE_K, std::make_tuple(0, 0, "Decrease linear speed")},
        {KEYCODE_L, std::make_tuple(0, 0, "Increase angular speed")},
        {KEYCODE_J, std::make_tuple(0, 0, "Decrease angular speed")},

        {KEYCODE_M, std::make_tuple(0, 0, "OpenLed")},
        {KEYCODE_N, std::make_tuple(0, 0, "OpenBuzzer")}

    };
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTeleop>("robot_teleop");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
