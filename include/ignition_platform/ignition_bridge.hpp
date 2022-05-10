#ifndef IGNITION_BRIDGE_HPP_
#define IGNITION_BRIDGE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include "ros_ign_bridge/convert.hpp"

#include <string>

namespace ignition_platform
{
    typedef void (*odometryCallbackType)(const nav_msgs::msg::Odometry &msg);

    class IgnitionBridge
    {
    public:
        IgnitionBridge(std::string name_space = "");
        ~IgnitionBridge(){};

    public:
        std::shared_ptr<ignition::transport::Node> ign_node_ptr_;

        ignition::msgs::Twist ign_msg_;
        ignition::transport::v11::Node::Publisher command_twist_pub_;
        
    private:
        const std::string &ign_topic_command_twist_ = "/cmd_vel";
        const std::string &ign_topic_sensor_odometry_ = "/pose";

    public:
        void sendTwistMsg(const geometry_msgs::msg::Twist &msg);
        void setOdometryCallback(odometryCallbackType callback);

    private:
        static odometryCallbackType odometryCallback_;
        static void ignitionOdometryCallback(const ignition::msgs::Odometry &msg);
        
    };
}

#endif // IGNITION_BRIDGE_HPP_