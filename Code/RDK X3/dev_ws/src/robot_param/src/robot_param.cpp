#include "robot_param/robot_param.h"

RobotParam::RobotParam(std::string nodeName) : Node(nodeName)
{
    Robot_Param *rp;
    // Declare parameters first
    descriptor.description = "";
    descriptor.name = "name";
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = 10;
    descriptor.integer_range[0].to_value = 1000;
    descriptor.integer_range[0].step = 1;
    this->declare_parameter<uint16_t>("do_pid_interval", rp->do_pid_interval);
    this->declare_parameter<uint16_t>("kp", rp->kp);
    this->declare_parameter<uint16_t>("ki", rp->ki);
    this->declare_parameter<uint16_t>("kd", rp->kd);
    // Then create callback
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RobotParam::SetParametersCallback, this, std::placeholders::_1, rp));
}

rcl_interfaces::msg::SetParametersResult RobotParam::SetParametersCallback(
        const std::vector<rclcpp::Parameter> &parameters, Robot_Param *rp)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (auto &param : parameters)
        {
            RCLCPP_INFO(this->get_logger(), "param %s update", param.get_name().c_str());
            if (param.get_name() == "do_pid_interval")
            {
                rp->do_pid_interval = param.as_int();
            }
            else if (param.get_name() == "kp")
            {
                rp->kp = param.as_int();
            }
            else if (param.get_name() == "ki")
            {
                rp->ki = param.as_int();
            }
            else if (param.get_name() == "kd")
            {
                rp->kd = param.as_int();
            }
        }

        // DataHolder::dump_params(rp);

        // param_update_flag_ = true;
        return result;
    }


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotParam>("robot_param");
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}