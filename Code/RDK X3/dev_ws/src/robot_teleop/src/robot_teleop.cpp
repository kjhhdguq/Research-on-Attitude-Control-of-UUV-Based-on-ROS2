#include "robot_teleop/robot_teleop.h"

// init启动
RobotTeleop::RobotTeleop(std::string nodeName) : Node(nodeName)
{
    RCLCPP_INFO(this->get_logger(), "Starting up OriginBot telop keyboard controller");

    _speed_linear_x = MAX_SPEED_LINEARE_X;
    _speed_angular_z = MAX_SPEED_ANGULAR_Z;

    tcgetattr(kfd, &initial_settings);
    new_settings = initial_settings;
    // 使用标准输入模式 | 显示输入字符
    new_settings.c_lflag &= ~(ICANON | ECHO);
    // VEOL: 附加的end of life字符
    new_settings.c_cc[VEOL] = 1;
    // VEOF: end of life字符
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);

    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    showMenu();
    teleopKeyboardLoop();
}

void RobotTeleop::stopRobot()
{
    cmdvel_.linear.x = 0.0;
    cmdvel_.angular.z = 0.0;
    pub_cmd->publish(cmdvel_);
}

void RobotTeleop::showMenu()
{
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "|    q   w   e   |   left-forward    forward     right-forward  |" << std::endl;
    std::cout << "|    a   s   d   |   left-turn       backward    right-turn     |" << std::endl;
    std::cout << "|    z       c   |   left_backward               right-backward |" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    std::cout << "i/k : increase/decrease linear velocity                         |" << std::endl;
    std::cout << "j/l : increase/decrease angular velocity                        |" << std::endl;

    std::cout << std::endl;
    std::cout << "press h to Menu and ctrl+c to quit" << std::endl;
}

void RobotTeleop::teleopKeyboardLoop()
{
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    char key;
    bool dirty = false;
    int speed = 0, turn = 0;

    while (rclcpp::ok())
    {
        boost::this_thread::interruption_point();
        int originbotBaseBit = 0;
        int ret;

        if ((ret = poll(&ufd, 1, 500)) < 0)
        {
            tcsetattr(kfd, TCSANOW, &initial_settings);
            perror("poll():");
            return;
        }
        else if (ret > 0)
        {
            new_settings.c_cc[VMIN] = 0;
            tcsetattr(0, TCSANOW, &new_settings);
            read(0, &key, 1);
            new_settings.c_cc[VMIN] = 1;
            tcsetattr(0, TCSANOW, &new_settings);
        }
        else
        {
            if (dirty)
            {
                stopRobot();
                dirty = false;
            }
            continue;
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
            {KEYCODE_J, std::make_tuple(0, 0, "Decrease angular speed")}};

        if (key == KEYCODE_H)
        {
            showMenu();
            continue;
        }
        if (key == KEYCODE_I)
        {
            _speed_linear_x += _speed_linear_x * 0.1;
            std::cout << "Linear speed increased to: " << _speed_linear_x << std::endl;
            continue;
        }
        if (key == KEYCODE_K)
        {
            _speed_linear_x -= _speed_linear_x * 0.1;
            std::cout << "Linear speed decrease to: " << _speed_linear_x << std::endl;
            continue;
        }
        if (key == KEYCODE_L)
        {
            _speed_angular_z += _speed_angular_z * 0.1;
            std::cout << "Angular speed increased to: " << _speed_angular_z << std::endl;
            continue;
        }
        if (key == KEYCODE_J)
        {
            _speed_angular_z -= _speed_angular_z * 0.1;
            std::cout << "Angular speed decrease to: " << _speed_angular_z << std::endl;
            continue;
        }

        auto it = keymap.find(key);
        if (it != keymap.end())
        {
            speed = std::get<0>(it->second);
            turn = std::get<1>(it->second);
            dirty = true;
            originbotBaseBit = 1;
            std::cout << std::get<2>(it->second) << std::endl;
        }
        else
        {
            speed = 0;
            turn = 0;
            dirty = false;
            originbotBaseBit = 1;
        }

        if (originbotBaseBit == 1)
        {
            cmdvel_.linear.x = speed * _speed_linear_x;
            cmdvel_.angular.z = turn * _speed_angular_z;
            pub_cmd->publish(cmdvel_);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTeleop>("robot_teleop");
    boost::thread thread = boost::thread(boost::bind(&RobotTeleop::teleopKeyboardLoop, node));
    rclcpp::spin(node);
    thread.interrupt();
    rclcpp::shutdown();
    return 0;
}
