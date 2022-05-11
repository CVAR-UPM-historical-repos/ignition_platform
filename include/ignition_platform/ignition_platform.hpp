#ifndef IGNITION_PLATFORM_HPP_
#define IGNITION_PLATFORM_HPP_

#include <memory>
#include <string>
#include <iostream>

#include <as2_core/core_functions.hpp>
#include <as2_core/aerial_platform.hpp>
#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_transport/image_transport.hpp>

#include "ignition_bridge.hpp"

namespace ignition_platform
{
    class IgnitionPlatform : public as2::AerialPlatform
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

        static std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> pose_ptr_;
        static void poseCallback(const geometry_msgs::msg::PoseStamped &msg);

        static std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
        static void odometryCallback(const nav_msgs::msg::Odometry &msg);

        static std::unique_ptr<as2::sensors::Camera> camera_ptr_;
        static void cameraCallback(const sensor_msgs::msg::Image &msg);

        static std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::CameraInfo>> camera_info_ptr_;
        static void cameraInfoCallback(const sensor_msgs::msg::CameraInfo &msg);

    private:
        std::shared_ptr<IgnitionBridge> ignition_bridge_;

    private:
        void resetCommandTwistMsg();
    };
}

#endif // IGNITION_PLATFORM_HPP_
