#include "ignition_platform.hpp"

namespace ignition_platform
{
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

    // IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
    IgnitionPlatform::IgnitionPlatform() : as2::Node(std::string("platform"))
    {   
        ignition_bridge_ = std::make_shared<IgnitionBridge>(this->get_namespace());

        twist_command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            this->generate_global_name("actuator_command/twist"), rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
            {
                this->command_twist_msg_ = *msg.get();
                this->ownSendCommand();
            });

        this->configureSensors();
    };

    void IgnitionPlatform::configureSensors()
    {
        odometry_raw_estimation_ptr_ =
            std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
        
        ignition_bridge_->setOdometryCallback(odometryCallback);

        return;
    };

    bool IgnitionPlatform::ownSendCommand()
    {
        ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
        return false;
    };

    bool IgnitionPlatform::ownSetArmingState(bool state)
    {
        return true;
    };

    bool IgnitionPlatform::ownSetOffboardControl(bool offboard)
    {
        return true;
    };

    bool IgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg)
    {
        return true;
    };

    void IgnitionPlatform::odometryCallback(const nav_msgs::msg::Odometry &odom_msg)
    {
        odometry_raw_estimation_ptr_->updateData(odom_msg);
        return;
    }
}