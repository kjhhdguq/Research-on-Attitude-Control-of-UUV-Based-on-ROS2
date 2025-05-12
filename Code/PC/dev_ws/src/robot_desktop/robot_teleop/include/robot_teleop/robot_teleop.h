#ifndef ORIGINBOT_TELOP_H
#define ORIGINBOT_TELOP_H

#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/msg/command.hpp"
#include "robot_interfaces/msg/status_control.hpp"
#include "robot_interfaces/msg/calibration.hpp"
#include "robot_interfaces/msg/mode.hpp"
#include "robot_interfaces/msg/motor.hpp"
#include "robot_interfaces/msg/pid.hpp"
#include "robot_interfaces/srv/robot_led.hpp"
#include "robot_interfaces/srv/robot_buzzer.hpp"

#include "rclcpp/rclcpp.hpp"
#include <boost/thread/thread.hpp>
#include <dirent.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termio.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <memory>

// 工作空间一定要定义
using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// 定义键盘按键键值
#define KEYCODE_A (0x61)
#define KEYCODE_B (0x62)
#define KEYCODE_C (0x63)
#define KEYCODE_D (0x64)
#define KEYCODE_E (0x65)
#define KEYCODE_F (0x66)
#define KEYCODE_G (0x67)
#define KEYCODE_H (0x68)
#define KEYCODE_I (0x69)
#define KEYCODE_J (0x6a)
#define KEYCODE_K (0x6b)
#define KEYCODE_L (0x6c)
#define KEYCODE_M (0x6d)
#define KEYCODE_N (0x6e)
#define KEYCODE_O (0x6f)
#define KEYCODE_P (0x70)
#define KEYCODE_Q (0x71)
#define KEYCODE_R (0x72)
#define KEYCODE_S (0x73)
#define KEYCODE_T (0x74)
#define KEYCODE_U (0x75)
#define KEYCODE_V (0x76)
#define KEYCODE_W (0x77)
#define KEYCODE_X (0x78)
#define KEYCODE_Y (0x79)
#define KEYCODE_Z (0x7A)

#define MAX_SPEED_LINEARE_X (0.5)
#define MAX_SPEED_ANGULAR_Z (0.5)


class RobotTeleop : public rclcpp::Node
{
private:
  rclcpp::Time current_time_;
  rclcpp::TimerBase::SharedPtr timer_100ms_;
  robot_interfaces::msg::Command motion_Cmd;
  robot_interfaces::msg::Mode motion_mode;
  robot_interfaces::msg::Motor motion_motor;
  robot_interfaces::msg::Calibration imu_calib;
  robot_interfaces::msg::StatusControl ledf_status;

  rclcpp::Client<robot_interfaces::srv::RobotLed>::SharedPtr led_client;
  rclcpp::Client<robot_interfaces::srv::RobotBuzzer>::SharedPtr buzzer_client;

  rclcpp::Publisher<robot_interfaces::msg::Command>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<robot_interfaces::msg::Mode>::SharedPtr mode_publisher_;
  rclcpp::Publisher<robot_interfaces::msg::Motor>::SharedPtr motor_publisher_;
  rclcpp::Publisher<robot_interfaces::msg::Calibration>::SharedPtr cali_publisher_;
  rclcpp::Publisher<robot_interfaces::msg::StatusControl>::SharedPtr ledf_publisher_;

  struct termios initial_settings, new_settings;
  int kfd = 0;

public:
  RobotTeleop(std::string nodeName);
  ~RobotTeleop()
  {
    tcsetattr(0, TCSANOW, &new_settings);
  };
  void showMenu();
  void stopRobot();
  void teleopKeyboardLoop();
  void SendLedRequest(void);
  void timer_100ms_callback();
  void SendBuzzerRequest(void);
  void SendLedfStatus(void);
  void SendMotionMode(void);
  void SendJY901Cali(void);
};

#endif // ORIGINBOT_TELOP_H
