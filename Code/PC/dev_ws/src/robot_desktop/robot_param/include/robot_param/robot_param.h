#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp> // ROS2 C++接口库
#include "robot_interfaces/msg/pid.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

typedef struct
{
    uint8_t pid_flag;
    uint8_t pid_interval;

    float pidx_kp;
    float pidx_ki;
    float pidx_kd;

    float pidy_kp;
    float pidy_ki;
    float pidy_kd;

    float pidz_kp;
    float pidz_ki;
    float pidz_kd;

    float pidroll_kp;
    float pidroll_ki;
    float pidroll_kd;

    float pidpitch_kp;
    float pidpitch_ki;
    float pidpitch_kd;

    float pidyaw_kp;
    float pidyaw_ki;
    float pidyaw_kd;
} Robot_Param;


class RobotParam : public rclcpp::Node
{
public:
    RobotParam(std::string nodeName);
    ~RobotParam()
    {
    }
    Robot_Param* rp;
    // void Robot_Pid_Param_Declare(void);

    rcl_interfaces::msg::SetParametersResult SetParametersCallback(
        const std::vector<rclcpp::Parameter> &parameters, Robot_Param &rp);

private:
    // rclcpp::SyncParametersClient::SharedPtr pid_paramClient;
    robot_interfaces::msg::PID pid_param_flag;
    robot_interfaces::msg::PID pid_param_interval;

    robot_interfaces::msg::PID pid_param_X;
    robot_interfaces::msg::PID pid_param_Y;
    robot_interfaces::msg::PID pid_param_Z;
    robot_interfaces::msg::PID pid_param_Roll;
    robot_interfaces::msg::PID pid_param_Pitch;
    robot_interfaces::msg::PID pid_param_Yaw;

    rclcpp::Publisher<robot_interfaces::msg::PID>::SharedPtr pid_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};