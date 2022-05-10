#include "ignition_bridge.hpp"

namespace ignition_platform
{
    odometryCallbackType IgnitionBridge::odometryCallback_ = [](const nav_msgs::msg::Odometry &msg){};

    IgnitionBridge::IgnitionBridge(std::string name_space)
    {
        // Initialize the ignition node
        ign_node_ptr_ = std::make_shared<ignition::transport::Node>();

        // Initialize publishers
        command_twist_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Twist>(
            name_space + ign_topic_command_twist_);

        std::cout << "Twist topic: " << name_space + ign_topic_command_twist_ << std::endl;

        // Initialize subscribers
        ign_node_ptr_->Subscribe(
            "model" + name_space + ign_topic_sensor_odometry_, 
            IgnitionBridge::ignitionOdometryCallback);
        
        return;
    };

    void IgnitionBridge::sendTwistMsg(const geometry_msgs::msg::Twist &ros_twist_msg)
    {
        ros_ign_bridge::convert_ros_to_ign(ros_twist_msg, ign_msg_);
        command_twist_pub_.Publish(ign_msg_);
        return;
    };

    void IgnitionBridge::setOdometryCallback(odometryCallbackType callback)
    {
        odometryCallback_ = callback;
        return;
    };

    void IgnitionBridge::ignitionOdometryCallback(const ignition::msgs::Odometry &msg)
    {
        nav_msgs::msg::Odometry odom_msg;
        ros_ign_bridge::convert_ign_to_ros(msg, odom_msg);
        odometryCallback_(odom_msg);
        return;
    };    
}