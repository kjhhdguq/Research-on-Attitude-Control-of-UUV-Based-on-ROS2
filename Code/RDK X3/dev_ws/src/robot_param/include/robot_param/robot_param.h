#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

typedef struct
{
    uint16_t do_pid_interval;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
} Robot_Param;

class RobotParam : public rclcpp::Node
{
public:
    RobotParam(std::string nodeName);
    ~RobotParam()
    {
    }

private:
    rcl_interfaces::msg::SetParametersResult SetParametersCallback(
        const std::vector<rclcpp::Parameter> &parameters, Robot_Param *rp);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::ParameterDescriptor descriptor;
};