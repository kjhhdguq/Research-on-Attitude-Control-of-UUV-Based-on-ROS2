#include "robot_param/robot_param.h"

RobotParam::RobotParam(std::string nodeName) : Node(nodeName)
{
    // 用一个结构体实时保存参数（这里不要用指针，要不然会可能占用非法内存）
    Robot_Param rp;

    // // 定义PID调参标志位，方便stm32下位机读写内部flash
    this->declare_parameter<bool>("PID_flag", 0);

    // 定义PID间隔时间参数
    auto pid_interval = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pid_interval_range = std::make_shared<rcl_interfaces::msg::IntegerRange>();
    pid_interval_range->from_value = 0; // 范围起始值
    pid_interval_range->to_value = 50; // 范围终止值
    pid_interval_range->step = 1;       // 步长
    pid_interval->integer_range.emplace_back(*pid_interval_range);
    pid_interval->description = "PID间隔时间";
    this->declare_parameter<uint8_t>("PID_interval", 0, *pid_interval);

    // 定义PIDX-kp参数
    auto pidx_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidx_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidx_kp_range->from_value = 0.0000;  // 范围起始值
    pidx_kp_range->to_value = 2000.0000; // 范围终止值
    pidx_kp_range->step = 0.0001;       // 步长
    pidx_kp->floating_point_range.emplace_back(*pidx_kp_range);
    pidx_kp->description = "PIDX参数kp";
    this->declare_parameter<double>("PIDX_kp", 0.0000, *pidx_kp);

    // 定义PIDX-ki参数
    auto pidx_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidx_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidx_ki_range->from_value = 0.0000;  // 范围起始值
    pidx_ki_range->to_value = 2000.0000; // 范围终止值
    pidx_ki_range->step = 0.0001;       // 步长
    pidx_ki->floating_point_range.emplace_back(*pidx_ki_range);
    pidx_ki->description = "PIDX参数ki";
    this->declare_parameter<double>("PIDX_ki", 0.0000, *pidx_ki);

    // 定义PIDX-kd参数
    auto pidx_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidx_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidx_kd_range->from_value = 0.0000;  // 范围起始值
    pidx_kd_range->to_value = 2000.0000; // 范围终止值
    pidx_kd_range->step = 0.0001;       // 步长
    pidx_kd->floating_point_range.emplace_back(*pidx_kd_range);
    pidx_kd->description = "PIDX参数kd";
    this->declare_parameter<double>("PIDX_kd", 0.0000, *pidx_kd);

    // 定义PIDY-kp参数
    auto pidy_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidy_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidy_kp_range->from_value = 0.0000;  // 范围起始值
    pidy_kp_range->to_value = 2000.0000; // 范围终止值
    pidy_kp_range->step = 0.0001;       // 步长
    pidy_kp->floating_point_range.emplace_back(*pidy_kp_range);
    pidy_kp->description = "PIDY参数kp";
    this->declare_parameter<double>("PIDY_kp", 0.0000, *pidy_kp);

    // 定义PIDY-ki参数
    auto pidy_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidy_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidy_ki_range->from_value = 0.0000;  // 范围起始值
    pidy_ki_range->to_value = 2000.0000; // 范围终止值
    pidy_ki_range->step = 0.0001;       // 步长
    pidy_ki->floating_point_range.emplace_back(*pidy_ki_range);
    pidy_ki->description = "PIDY参数ki";
    this->declare_parameter<double>("PIDY_ki", 0.0000, *pidy_ki);

    // 定义PIDY-kd参数
    auto pidy_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidy_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidy_kd_range->from_value = 0.0000;  // 范围起始值
    pidy_kd_range->to_value = 2000.0000; // 范围终止值
    pidy_kd_range->step = 0.0001;       // 步长
    pidy_kd->floating_point_range.emplace_back(*pidy_kd_range);
    pidy_kd->description = "PIDY参数kd";
    this->declare_parameter<double>("PIDY_kd", 0.0000, *pidy_kd);

    // 定义PIDZ-kp参数
    auto pidz_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidz_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidz_kp_range->from_value = 0.0000;  // 范围起始值
    pidz_kp_range->to_value = 2000.0000; // 范围终止值
    pidz_kp_range->step = 0.0001;       // 步长
    pidz_kp->floating_point_range.emplace_back(*pidz_kp_range);
    pidz_kp->description = "PIDZ参数kp";
    this->declare_parameter<double>("PIDZ_kp", 0.0000, *pidz_kp);

    // 定义PIDZ-ki参数
    auto pidz_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidz_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidz_ki_range->from_value = 0.0000;  // 范围起始值
    pidz_ki_range->to_value = 2000.0000; // 范围终止值
    pidz_ki_range->step = 0.0001;       // 步长
    pidz_ki->floating_point_range.emplace_back(*pidz_ki_range);
    pidz_ki->description = "PIDZ参数ki";
    this->declare_parameter<double>("PIDZ_ki", 0.0000, *pidz_ki);

    // 定义PIDZ-kd参数
    auto pidz_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidz_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidz_kd_range->from_value = 0.0000;  // 范围起始值
    pidz_kd_range->to_value = 2000.0000; // 范围终止值
    pidz_kd_range->step = 0.0001;       // 步长
    pidz_kd->floating_point_range.emplace_back(*pidz_kd_range);
    pidz_kd->description = "PIDZ参数kd";
    this->declare_parameter<double>("PIDZ_kd", 0.0000, *pidz_kd);

    // 定义PIDRoll-kp参数
    auto pidroll_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidroll_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidroll_kp_range->from_value = 0.0000;  // 范围起始值
    pidroll_kp_range->to_value = 2000.0000; // 范围终止值
    pidroll_kp_range->step = 0.0001;       // 步长
    pidroll_kp->floating_point_range.emplace_back(*pidroll_kp_range);
    pidroll_kp->description = "PIDRoll参数kp";
    this->declare_parameter<double>("PIDRoll_kp", 0.0000, *pidroll_kp);

    // 定义PIDRoll-ki参数
    auto pidroll_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidroll_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidroll_ki_range->from_value = 0.0000;  // 范围起始值
    pidroll_ki_range->to_value = 2000.0000; // 范围终止值
    pidroll_ki_range->step = 0.0001;       // 步长
    pidroll_ki->floating_point_range.emplace_back(*pidroll_ki_range);
    pidroll_ki->description = "PIDRoll参数ki";
    this->declare_parameter<double>("PIDRoll_ki", 0.0000, *pidroll_ki);

    // 定义PIDRoll-kd参数
    auto pidroll_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidroll_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidroll_kd_range->from_value = 0.0000;  // 范围起始值
    pidroll_kd_range->to_value = 2000.0000; // 范围终止值
    pidroll_kd_range->step = 0.0001;       // 步长
    pidroll_kd->floating_point_range.emplace_back(*pidroll_kd_range);
    pidroll_kd->description = "PIDRoll参数kd";
    this->declare_parameter<double>("PIDRoll_kd", 0.0000, *pidroll_kd);

    // 定义PIDPitch-kp参数
    auto pidpitch_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidpitch_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidpitch_kp_range->from_value = 0.0000;  // 范围起始值
    pidpitch_kp_range->to_value = 2000.0000; // 范围终止值
    pidpitch_kp_range->step = 0.0001;       // 步长
    pidpitch_kp->floating_point_range.emplace_back(*pidpitch_kp_range);
    pidpitch_kp->description = "PIDPitch参数kp";
    this->declare_parameter<double>("PIDPitch_kp", 0.0000, *pidpitch_kp);

    // 定义PIDPitch-ki参数
    auto pidpitch_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidpitch_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidpitch_ki_range->from_value = 0.0000;  // 范围起始值
    pidpitch_ki_range->to_value = 2000.0000; // 范围终止值
    pidpitch_ki_range->step = 0.0001;       // 步长
    pidpitch_ki->floating_point_range.emplace_back(*pidpitch_ki_range);
    pidpitch_ki->description = "PIDPitch参数ki";
    this->declare_parameter<double>("PIDPitch_ki", 0.0000, *pidpitch_ki);

    // 定义PIDPitch-kd参数
    auto pidpitch_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidpitch_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidpitch_kd_range->from_value = 0.0000;  // 范围起始值
    pidpitch_kd_range->to_value = 2000.0000; // 范围终止值
    pidpitch_kd_range->step = 0.0001;       // 步长
    pidpitch_kd->floating_point_range.emplace_back(*pidpitch_kd_range);
    pidpitch_kd->description = "PIDPitch参数kd";
    this->declare_parameter<double>("PIDPitch_kd", 0.0000, *pidpitch_kd);

    // 定义PIDYaw-kp参数
    auto pidyaw_kp = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidyaw_kp_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidyaw_kp_range->from_value = 0.0000;  // 范围起始值
    pidyaw_kp_range->to_value = 2000.0000; // 范围终止值
    pidyaw_kp_range->step = 0.0001;       // 步长
    pidyaw_kp->floating_point_range.emplace_back(*pidyaw_kp_range);
    pidyaw_kp->description = "PIDYaw参数kp";
    this->declare_parameter<double>("PIDYaw_kp", 0.0000, *pidyaw_kp);

    // 定义PIDYaw-ki参数
    auto pidyaw_ki = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidyaw_ki_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidyaw_ki_range->from_value = 0.0000;  // 范围起始值
    pidyaw_ki_range->to_value = 2000.0000; // 范围终止值
    pidyaw_ki_range->step = 0.0001;       // 步长
    pidyaw_ki->floating_point_range.emplace_back(*pidyaw_ki_range);
    pidyaw_ki->description = "PIDYaw参数ki";
    this->declare_parameter<double>("PIDYaw_ki", 0.0000, *pidyaw_ki);

    // 定义PIDYaw-kd参数
    auto pidyaw_kd = std::make_shared<rcl_interfaces::msg::ParameterDescriptor>();
    auto pidyaw_kd_range = std::make_shared<rcl_interfaces::msg::FloatingPointRange>();
    pidyaw_kd_range->from_value = 0.0000;  // 范围起始值
    pidyaw_kd_range->to_value = 2000.0000; // 范围终止值
    pidyaw_kd_range->step = 0.0001;       // 步长
    pidyaw_kd->floating_point_range.emplace_back(*pidyaw_kd_range);
    pidyaw_kd->description = "PIDYaw参数kd";
    this->declare_parameter<double>("PIDYaw_kd", 0.0000, *pidyaw_kd);

    pid_publisher_ = this->create_publisher<robot_interfaces::msg::PID>("robot_pid", 10);

    // 初始化原先的pid参数
    bool pid_param_flag_init;
    pid_param_flag.id = 0;
    this->get_parameter("PID_flag", pid_param_flag_init);
    pid_param_flag.pid_flag = (uint8_t)pid_param_flag_init;
    pid_publisher_->publish(pid_param_flag);

    pid_param_interval.id = 7;
    this->get_parameter("PID_interval", pid_param_interval.pid_interval);
    pid_publisher_->publish(pid_param_interval);

    pid_param_X.id = 1;
    this->get_parameter("PIDX_kp", pid_param_X.p);
    this->get_parameter("PIDX_ki", pid_param_X.i);
    this->get_parameter("PIDX_kd", pid_param_X.d);
    pid_publisher_->publish(pid_param_X);

    pid_param_Y.id = 2;
    this->get_parameter("PIDY_kp", pid_param_Y.p);
    this->get_parameter("PIDY_ki", pid_param_Y.i);
    this->get_parameter("PIDY_kd", pid_param_Y.d);
    pid_publisher_->publish(pid_param_Y);

    pid_param_Z.id = 3;
    this->get_parameter("PIDZ_kp", pid_param_Z.p);
    this->get_parameter("PIDZ_ki", pid_param_Z.i);
    this->get_parameter("PIDZ_kd", pid_param_Z.d);
    pid_publisher_->publish(pid_param_Z);

    pid_param_Roll.id = 4;
    this->get_parameter("PIDRoll_kp", pid_param_Roll.p);
    this->get_parameter("PIDRoll_ki", pid_param_Roll.i);
    this->get_parameter("PIDRoll_kd", pid_param_Roll.d);
    pid_publisher_->publish(pid_param_Roll);

    pid_param_Pitch.id = 5;
    this->get_parameter("PIDPitch_kp", pid_param_Pitch.p);
    this->get_parameter("PIDPitch_ki", pid_param_Pitch.i);
    this->get_parameter("PIDPitch_kd", pid_param_Pitch.d);
    pid_publisher_->publish(pid_param_Pitch);

    pid_param_Yaw.id = 6;
    this->get_parameter("PIDYaw_kp", pid_param_Yaw.p);
    this->get_parameter("PIDYaw_ki", pid_param_Yaw.i);
    this->get_parameter("PIDYaw_kd", pid_param_Yaw.d);
    pid_publisher_->publish(pid_param_Yaw);

    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RobotParam::SetParametersCallback, this, _1, rp));
    // pid_publisher_->publish(pid_param);
    // pid_paramClient = std::make_shared<rclcpp::SyncParametersClient>(this, "pid_param_service");
}

// 此处使用引用
rcl_interfaces::msg::SetParametersResult RobotParam::SetParametersCallback(
    const std::vector<rclcpp::Parameter> &parameters, Robot_Param &rp)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (auto &param : parameters)
    {
        // PID-Flag
        if (param.get_name() == "PID_flag")
        {
            pid_param_flag.id = 0;
            rp.pid_flag = param.as_bool();
            pid_param_flag.pid_flag = rp.pid_flag;
            pid_publisher_->publish(pid_param_flag);
            RCLCPP_INFO(this->get_logger(), "PID_flag update to %d", pid_param_flag.pid_flag);
            pid_publisher_->publish(pid_param_X);
            pid_publisher_->publish(pid_param_Y);
            pid_publisher_->publish(pid_param_Z);
            pid_publisher_->publish(pid_param_Roll);
            pid_publisher_->publish(pid_param_Pitch);
            pid_publisher_->publish(pid_param_Yaw);
            RCLCPP_INFO(this->get_logger(), "Begin to update PID");
        }

        // PID-Interval
        else if (param.get_name() == "PID_interval")
        {
            pid_param_interval.id = 7;
            rp.pid_interval = param.as_int();
            pid_param_interval.pid_interval = rp.pid_interval;
            pid_publisher_->publish(pid_param_interval);
            RCLCPP_INFO(this->get_logger(), "PID_interval update to %d", pid_param_interval.pid_interval);
        }

        // PIDX
        else if (param.get_name() == "PIDX_kp")
        {
            pid_param_X.id = 1;
            rp.pidx_kp = param.as_double();
            pid_param_X.p = rp.pidx_kp;
            pid_publisher_->publish(pid_param_X);
            RCLCPP_INFO(this->get_logger(), "PIDX_kp update to %0.4f", pid_param_X.p);
        }
        else if (param.get_name() == "PIDX_ki")
        {
            pid_param_X.id = 1;
            rp.pidx_ki = param.as_double();
            pid_param_X.i = rp.pidx_ki;
            pid_publisher_->publish(pid_param_X);
            RCLCPP_INFO(this->get_logger(), "PIDX_ki update to %0.4f", pid_param_X.i);
        }
        else if (param.get_name() == "PIDX_kd")
        {
            pid_param_X.id = 1;
            rp.pidx_kd = param.as_double();
            pid_param_X.d = rp.pidx_kd;
            pid_publisher_->publish(pid_param_X);
            RCLCPP_INFO(this->get_logger(), "PIDX_kd update to %0.4f", pid_param_X.d);
        }

        // PIDY
        else if (param.get_name() == "PIDY_kp")
        {
            pid_param_Y.id = 2;
            rp.pidy_kp = param.as_double();
            pid_param_Y.p = rp.pidy_kp;
            pid_publisher_->publish(pid_param_Y);
            RCLCPP_INFO(this->get_logger(), "PIDY_kp update to %0.4f", pid_param_Y.p);
        }
        else if (param.get_name() == "PIDY_ki")
        {
            pid_param_Y.id = 2;
            rp.pidy_ki = param.as_double();
            pid_param_Y.i = rp.pidy_ki;
            pid_publisher_->publish(pid_param_Y);
            RCLCPP_INFO(this->get_logger(), "PIDY_ki update to %0.4f", pid_param_Y.i);
        }
        else if (param.get_name() == "PIDY_kd")
        {
            pid_param_Y.id = 2;
            rp.pidy_kd = param.as_double();
            pid_param_Y.d = rp.pidy_kd;
            pid_publisher_->publish(pid_param_Y);
            RCLCPP_INFO(this->get_logger(), "PIDY_kd update to %0.4f", pid_param_Y.d);
        }

        // PIDZ
        else if (param.get_name() == "PIDZ_kp")
        {
            pid_param_Z.id = 3;
            rp.pidz_kp = param.as_double();
            pid_param_Z.p = rp.pidz_kp;
            pid_publisher_->publish(pid_param_Z);
            RCLCPP_INFO(this->get_logger(), "PIDZ_kp update to %0.4f", pid_param_Z.p);
        }
        else if (param.get_name() == "PIDZ_ki")
        {
            pid_param_Z.id = 3;
            rp.pidz_ki = param.as_double();
            pid_param_Z.i = rp.pidz_ki;
            pid_publisher_->publish(pid_param_Z);
            RCLCPP_INFO(this->get_logger(), "PIDZ_ki update to %0.4f", pid_param_Z.i);
        }
        else if (param.get_name() == "PIDZ_kd")
        {
            pid_param_Z.id = 3;
            rp.pidz_kd = param.as_double();
            pid_param_Z.d = rp.pidz_kd;
            pid_publisher_->publish(pid_param_Z);
            RCLCPP_INFO(this->get_logger(), "PIDZ_kd update to %0.4f", pid_param_Z.d);
        }

        // PIDRoll
        else if (param.get_name() == "PIDRoll_kp")
        {
            pid_param_Roll.id = 4;
            rp.pidroll_kp = param.as_double();
            pid_param_Roll.p = rp.pidroll_kp;
            pid_publisher_->publish(pid_param_Roll);
            RCLCPP_INFO(this->get_logger(), "PIDRoll_kp update to %0.4f", pid_param_Roll.p);
        }
        else if (param.get_name() == "PIDRoll_ki")
        {
            pid_param_Roll.id = 4;
            rp.pidroll_ki = param.as_double();
            pid_param_Roll.i = rp.pidroll_ki;
            pid_publisher_->publish(pid_param_Roll);
            RCLCPP_INFO(this->get_logger(), "PIDRoll_ki update to %0.4f", pid_param_Roll.i);
        }
        else if (param.get_name() == "PIDRoll_kd")
        {
            pid_param_Roll.id = 4;
            rp.pidroll_kd = param.as_double();
            pid_param_Roll.d = rp.pidroll_kd;
            pid_publisher_->publish(pid_param_Roll);
            RCLCPP_INFO(this->get_logger(), "PIDRoll_kd update to %0.4f", pid_param_Roll.d);
        }

        // PIDPitch
        else if (param.get_name() == "PIDPitch_kp")
        {
            pid_param_Pitch.id = 5;
            rp.pidpitch_kp = param.as_double();
            pid_param_Pitch.p = rp.pidpitch_kp;
            pid_publisher_->publish(pid_param_Pitch);
            RCLCPP_INFO(this->get_logger(), "PIDPitch_kp update to %0.4f", pid_param_Pitch.p);
        }
        else if (param.get_name() == "PIDPitch_ki")
        {
            pid_param_Pitch.id = 5;
            rp.pidpitch_ki = param.as_double();
            pid_param_Pitch.i = rp.pidpitch_ki;
            pid_publisher_->publish(pid_param_Pitch);
            RCLCPP_INFO(this->get_logger(), "PIDPitch_ki update to %0.4f", pid_param_Pitch.i);
        }
        else if (param.get_name() == "PIDPitch_kd")
        {
            pid_param_Pitch.id = 5;
            rp.pidpitch_kd = param.as_double();
            pid_param_Pitch.d = rp.pidpitch_kd;
            pid_publisher_->publish(pid_param_Pitch);
            RCLCPP_INFO(this->get_logger(), "PIDPitch_kd update to %0.4f", pid_param_Pitch.d);
        }

        // PIDYaw
        else if (param.get_name() == "PIDYaw_kp")
        {
            pid_param_Yaw.id = 6;
            rp.pidyaw_kp = param.as_double();
            pid_param_Yaw.p = rp.pidyaw_kp;
            pid_publisher_->publish(pid_param_Yaw);
            RCLCPP_INFO(this->get_logger(), "PIDYaw_kp update to %0.4f", pid_param_Yaw.p);
        }
        else if (param.get_name() == "PIDYaw_ki")
        {
            pid_param_Yaw.id = 6;
            rp.pidyaw_ki = param.as_double();
            pid_param_Yaw.i = rp.pidyaw_ki;
            pid_publisher_->publish(pid_param_Yaw);
            RCLCPP_INFO(this->get_logger(), "PIDYaw_ki update to %0.4f", pid_param_Yaw.i);
        }
        else if (param.get_name() == "PIDYaw_kd")
        {
            pid_param_Yaw.id = 6;
            rp.pidyaw_kd = param.as_double();
            pid_param_Yaw.d = rp.pidyaw_kd;
            pid_publisher_->publish(pid_param_Yaw);
            RCLCPP_INFO(this->get_logger(), "PIDYaw_kd update to %0.4f", pid_param_Yaw.d);
        }
    }

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