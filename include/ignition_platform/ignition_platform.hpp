#ifndef IGNITION_PLATFORM_HPP_
#define IGNITION_PLATFORM_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <as2_core/core_functions.hpp>
#include <as2_core/aerial_platform.hpp>
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

#include "ignition_bridge.hpp"

#define CMD_FREQ 10 // miliseconds
#define THRUST_THRESHOLD 0.001
#define THRUST_MIN 0.15f

namespace ignition_platform
{
    //class IgnitionPlatform : public as2::AerialPlatform
    class IgnitionPlatform : public as2::Node
    {
    public:
        IgnitionPlatform();
        ~IgnitionPlatform(){};

    public:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;
    geometry_msgs::msg::TwistStamped command_twist_msg_;

    

    public:
        void configureSensors();
        bool ownSendCommand();
        bool ownSetArmingState(bool state);
        bool ownSetOffboardControl(bool offboard);
        bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg);

        static std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
        static void odometryCallback(const nav_msgs::msg::Odometry &msg);

    private:
        std::shared_ptr<IgnitionBridge> ignition_bridge_;
    };
}

#endif // IGNITION_PLATFORM_HPP_
