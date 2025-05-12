#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <thread>
#include <algorithm>
#include <csignal>
#include <stdlib.h>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/msg/imu.hpp"

// 注意创建新的msg时所有变量一定是小写，且msg的开头首字母一定是大写字母否则编译报错
// 如果出现两个大写字母，中间要用下划线隔开否则报错
#include "robot_interfaces/msg/command.hpp"
#include "robot_interfaces/msg/imu.hpp"
#include "robot_interfaces/msg/status_control.hpp"
#include "robot_interfaces/msg/calibration.hpp"
#include "robot_interfaces/msg/mode.hpp"
#include "robot_interfaces/msg/motor.hpp"
#include "robot_interfaces/msg/pid.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_led.hpp"
#include "robot_interfaces/srv/robot_buzzer.hpp"
#include "robot_interfaces/srv/robot_pid.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define ORIGINBOT_WHEEL_TRACK  (0.11)

// originbot protocol data format
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    uint8_t data[6];
    uint8_t check;
    uint8_t tail;
} DataFrame;

// 8数据位数据帧
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    uint8_t data[8];
    uint8_t check;
    uint8_t tail;
} DataFrame_8;

// 12数据位数据帧
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    uint8_t data[12];
    uint8_t check;
    uint8_t tail;
} DataFrame_12;

typedef struct {
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_x;
    float angular_y;
    float angular_z;
    float roll;
    float pitch;
    float yaw;
} DataImu;

typedef struct {
    float battery_voltage;
    bool buzzer_on;
    bool led_on;
} RobotStatus;

enum {
    FRAME_ID_MOTION       = 0x01,
    FRAME_ID_VELOCITY     = 0x02,
    FRAME_ID_ACCELERATION = 0x03,
    FRAME_ID_ANGULAR      = 0x04,
    FRAME_ID_EULER        = 0x05,
    FRAME_ID_SENSOR       = 0x06,
    FRAME_ID_HMI          = 0x07,

    FRAME_ID_PIDGyro      = 0x13
};

class RobotBase : public rclcpp::Node
{
public:
    RobotBase(std::string nodeName);
    ~RobotBase();
    
private:
    void readRawData();
    bool checkDataFrame(DataFrame &frame);
    void processDataFrame(DataFrame &frame);

    void processVelocityData(DataFrame &frame);
    void processAngularData(DataFrame &frame);
    void processAccelerationData(DataFrame &frame);
    void processEulerData(DataFrame &frame);
    void processSensorData(DataFrame &frame);
    void processPidGyroData(DataFrame &frame);

    double imu_conversion(uint8_t data_high, uint8_t data_low);
    bool   imu_calibration();
    double degToRad(double deg);

    // void odom_publisher(float vx, float vth);
    void imu_publisher();
    
    bool buzzer_control(bool on);
    bool led_control(bool on);

    void cmd_vel_callback(const robot_interfaces::msg::Command &cmd);
    void mode_vel_callback(const robot_interfaces::msg::Mode &mode);
    void motor_vel_callback(const robot_interfaces::msg::Motor &motor);
    void pid_callback(const robot_interfaces::msg::PID &pid);
    void cali_callback(const robot_interfaces::msg::Calibration &cali);
    void ledf_callback(const robot_interfaces::msg::StatusControl &ledf);

    void buzzer_callback(const std::shared_ptr<robot_interfaces::srv::RobotBuzzer::Request>  request,
                               std::shared_ptr<robot_interfaces::srv::RobotBuzzer::Response> response);
    void led_callback(const std::shared_ptr<robot_interfaces::srv::RobotLed::Request>  request,
                            std::shared_ptr<robot_interfaces::srv::RobotLed::Response> response);

    void timer_100ms_callback();

private:
    serial::Serial serial_;
    rclcpp::Time current_time_;
    
    std::shared_ptr<std::thread> read_data_thread_;

    float correct_factor_vx_ = 1.0;
    float correct_factor_vth_ = 1.0;    

    DataImu imu_data_;
    RobotStatus robot_status_;

    bool use_imu_ = false;

    rclcpp::TimerBase::SharedPtr timer_100ms_;

    bool auto_stop_on_ = true;
    unsigned int auto_stop_count_ = 0;

    rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_publisher_;
    rclcpp::Subscription<robot_interfaces::msg::Command>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<robot_interfaces::msg::Mode>::SharedPtr mode_vel_subscription_;
    rclcpp::Subscription<robot_interfaces::msg::Motor>::SharedPtr motor_vel_subscription_;
    rclcpp::Subscription<robot_interfaces::msg::PID>::SharedPtr pid_subscription_;
    rclcpp::Subscription<robot_interfaces::msg::Calibration>::SharedPtr cali_vel_subscription_;
    rclcpp::Subscription<robot_interfaces::msg::StatusControl>::SharedPtr ledf_vel_subscription_;
    rclcpp::Publisher<robot_interfaces::msg::Imu>::SharedPtr imu_publisher_;

    // rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr status_publisher_;

    rclcpp::Service<robot_interfaces::srv::RobotBuzzer>::SharedPtr buzzer_service_;
    rclcpp::Service<robot_interfaces::srv::RobotLed>::SharedPtr led_service_;

};