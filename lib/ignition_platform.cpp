#include "ignition_platform.hpp"

namespace ignition_platform
{
    std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> IgnitionPlatform::pose_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Camera> IgnitionPlatform::camera_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::CameraInfo>> IgnitionPlatform::camera_info_ptr_ = nullptr;

    IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
    {   
        ignition_bridge_ = std::make_shared<IgnitionBridge>(this->get_namespace());
        this->configureSensors();
    };

    void IgnitionPlatform::configureSensors()
    {  
        pose_ptr_ =
            std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>("pose", this);
        ignition_bridge_->setPoseCallback(poseCallback);

        odometry_raw_estimation_ptr_ =
            std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
        ignition_bridge_->setOdometryCallback(odometryCallback);

        camera_ptr_ =
            std::make_unique<as2::sensors::Camera>("image", this);
        ignition_bridge_->setCameraCallback(cameraCallback);

        camera_info_ptr_ =
            std::make_unique<as2::sensors::Sensor<sensor_msgs::msg::CameraInfo>>("camera_info", this);
        ignition_bridge_->setCameraInfoCallback(cameraInfoCallback);

        return;
    };

    bool IgnitionPlatform::ownSendCommand()
    {
        ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
        return false;
    };

    bool IgnitionPlatform::ownSetArmingState(bool state)
    {
        resetCommandTwistMsg();
        return true;
    };

    bool IgnitionPlatform::ownSetOffboardControl(bool offboard)
    {
        resetCommandTwistMsg();
        return true;
    };

    bool IgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in)
    {
        if (control_in.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED &&
            control_in.control_mode == as2_msgs::msg::ControlMode::SPEED &&
            control_in.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME )
        {
            resetCommandTwistMsg();
            return true;
        }
        return false;
    };

    void IgnitionPlatform::resetCommandTwistMsg()
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.0f;
        twist_msg.linear.y = 0.0f;
        twist_msg.linear.z = 0.0f;
        twist_msg.angular.x = 0.0f;
        twist_msg.angular.y = 0.0f;
        twist_msg.angular.z = 0.0f;

        ignition_bridge_->sendTwistMsg(twist_msg);
    }

    void IgnitionPlatform::poseCallback(const geometry_msgs::msg::PoseStamped &pose_msg)
    {
        pose_ptr_->updateData(pose_msg);
        return;
    };

    void IgnitionPlatform::odometryCallback(const nav_msgs::msg::Odometry &odom_msg)
    {
        odometry_raw_estimation_ptr_->updateData(odom_msg);
        return;
    };

    void IgnitionPlatform::cameraCallback(const sensor_msgs::msg::Image &image_msg)
    {
        camera_ptr_->updateData(image_msg);
        return;
    };

    void IgnitionPlatform::cameraInfoCallback(const sensor_msgs::msg::CameraInfo &info_msg)
    {
        camera_info_ptr_->updateData(info_msg);
        return;
    };
}