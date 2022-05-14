#ifndef IGNITION_PLATFORM_HPP_
#define IGNITION_PLATFORM_HPP_

#include <memory>
#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

#include <math.h>

#include <as2_core/core_functions.hpp>
#include <as2_core/aerial_platform.hpp>
#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_transport/image_transport.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ignition_bridge.hpp"

#define CMD_FREQ 10  // miliseconds

namespace ignition_platform
{
    using Vector3d = Eigen::Vector3d;

    class IgnitionPlatform : public as2::AerialPlatform
    {
    public:
        IgnitionPlatform();
        ~IgnitionPlatform(){};

    public:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_command_sub_;

    public:
        void configureSensors();
        bool ownSendCommand() override;
        bool ownSetArmingState(bool state);
        bool ownSetOffboardControl(bool offboard);
        bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg);

        static std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> pose_ptr_;
        static void poseCallback(const geometry_msgs::msg::PoseStamped &msg);

        static std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
        static void odometryCallback(const nav_msgs::msg::Odometry &msg);

        static std::unique_ptr<as2::sensors::Camera> camera_ptr_;
        static void cameraCallback(const sensor_msgs::msg::Image &msg);
        static void cameraInfoCallback(const sensor_msgs::msg::CameraInfo &msg);

    private:
        std::shared_ptr<IgnitionBridge> ignition_bridge_;
        static bool camera_info_received_;
        static bool odometry_info_received_;
        as2_msgs::msg::ControlMode control_in_;
        static double yaw_;
        double yaw_rate_limit_ = M_PI_2;

    private:
        void resetCommandTwistMsg();
        Eigen::Vector3d convertENUtoFLU(const float yaw_angle, Eigen::Vector3d &enu_vec);
    };
}

#endif // IGNITION_PLATFORM_HPP_
