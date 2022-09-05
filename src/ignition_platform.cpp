/*!*******************************************************************************************
 *  \file       ignition_platform.cpp
 *  \brief      Implementation of an Ignition Gazebo UAV platform
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "ignition_platform.hpp"

namespace ignition_platform
{
std::shared_ptr<IgnitionBridge> IgnitionPlatform::ignition_bridge_ = nullptr;
bool IgnitionPlatform::odometry_info_received_ = false;
bool IgnitionPlatform::imu_info_received_ = false;
geometry_msgs::msg::Quaternion IgnitionPlatform::self_orientation_ = geometry_msgs::msg::Quaternion();
std::string IgnitionPlatform::namespace_ = "";

std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr IgnitionPlatform::ground_truth_twist_pub_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_usv_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr IgnitionPlatform::ground_truth_twist_usv_pub_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_targetA_pub_ =
    nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_targetB_pub_ =
    nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_targetC_pub_ =
    nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_targetD_pub_ =
    nullptr;

std::unique_ptr<sensor_msgs::msg::Imu> IgnitionPlatform::imu_msg_ = nullptr;

rclcpp::Clock::SharedPtr IgnitionPlatform::clock_ = nullptr;

IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
{
    clock_ = this->get_clock();

    this->declare_parameter<std::string>("world");
    std::string world_param = this->get_parameter("world").as_string();
    RCLCPP_INFO(this->get_logger(), "World: %s", world_param.c_str());

    namespace_ = this->get_namespace();

    ignition_bridge_ = std::make_shared<IgnitionBridge>(namespace_, world_param);

    this->configureSensors();

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(namespace_ + "/cmd_vel", rclcpp::QoS(1));

    ground_truth_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos);

    ground_truth_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos);

    ground_truth_pose_usv_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/usv/ground_truth/pose", as2_names::topics::ground_truth::qos);

    ground_truth_twist_usv_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/usv/ground_truth/twist", as2_names::topics::ground_truth::qos);

    ground_truth_pose_targetA_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "VesselTarget", as2_names::topics::ground_truth::qos);

    ground_truth_pose_targetB_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "box0/ground_truth/pose", as2_names::topics::ground_truth::qos);

    ground_truth_pose_targetC_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "box1/ground_truth/pose", as2_names::topics::ground_truth::qos);

    ground_truth_pose_targetD_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "box2/ground_truth/pose", as2_names::topics::ground_truth::qos);

    this->declare_parameter<std::string>("imu_topic");
    std::string imu_topic_param = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        this->generate_global_name(imu_topic_param), as2_names::topics::sensor_measurements::qos,
        std::bind(&IgnitionPlatform::imuCallback, this, std::placeholders::_1));

    // Timer to send command
    static auto timer_commands_ =
        this->create_wall_timer(std::chrono::milliseconds(CMD_FREQ), [this]() { this->sendCommand(); });
};

void IgnitionPlatform::configureSensors()
{
    ignition_bridge_->setUSVCallback(usvCallback);
    ignition_bridge_->setTargetACallback(targetACallback);
    ignition_bridge_->setTargetBCallback(targetBCallback);
    ignition_bridge_->setTargetCCallback(targetCCallback);
    ignition_bridge_->setTargetDCallback(targetDCallback);

    this->declare_parameter<bool>("use_odom_plugin");
    bool use_odom_plugin_param = this->get_parameter("use_odom_plugin").as_bool();

    if (use_odom_plugin_param)
    {
        odometry_raw_estimation_ptr_ = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
        ignition_bridge_->setOdometryCallback(odometryCallback);
    }

    this->declare_parameter<bool>("use_ground_truth");
    bool use_ground_truth_param = this->get_parameter("use_ground_truth").as_bool();
    if (use_ground_truth_param)
    {
        ignition_bridge_->setGroundTruthCallback(groundTruthCallback);
    }

    if (!use_odom_plugin_param && !use_ground_truth_param)
    {
        RCLCPP_WARN(this->get_logger(), "Using external odometry");
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
            std::bind(&IgnitionPlatform::poseCallback, this, std::placeholders::_1));
    }

    return;
};

bool IgnitionPlatform::ownSendCommand()
{
    if (control_in_.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME)
    {
        if (!odometry_info_received_)
        {
            return false;
        }
        odometry_info_received_ = false;

        if (command_twist_msg_.twist.angular.z > yaw_rate_limit_)
        {
            command_twist_msg_.twist.angular.z = yaw_rate_limit_;
        }
        else if (command_twist_msg_.twist.angular.z < -yaw_rate_limit_)
        {
            command_twist_msg_.twist.angular.z = -yaw_rate_limit_;
        }

        Eigen::Vector3d twist_lineal_enu = Eigen::Vector3d(
            command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y, command_twist_msg_.twist.linear.z);

        Eigen::Vector3d twist_lineal_flu = as2::FrameUtils::convertENUtoFLU(self_orientation_, twist_lineal_enu);
        command_twist_msg_.twist.linear.x = twist_lineal_flu(0);
        command_twist_msg_.twist.linear.y = twist_lineal_flu(1);
        command_twist_msg_.twist.linear.z = twist_lineal_flu(2);

        // ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
        twist_pub_->publish(command_twist_msg_.twist);
    }
    else if (control_in_.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME)
    {
        // ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
        twist_pub_->publish(command_twist_msg_.twist);
    }
    return true;
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
        (control_in.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME ||
         control_in.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME))
    {
        control_in_ = control_in;
        resetCommandTwistMsg();
        return true;
    }

    RCLCPP_WARN(this->get_logger(), "IgnitionPlatform::ownSetPlatformControlMode() - unsupported control mode");
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

    // twist_pub_->publish(twist_msg);
    // ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
}

void IgnitionPlatform::odometryCallback(nav_msgs::msg::Odometry &odom_msg)
{
    odom_msg.header.frame_id = generateTfName(namespace_, "odom");

    odometry_raw_estimation_ptr_->updateData(odom_msg);

    self_orientation_ = odom_msg.pose.pose.orientation;
    odometry_info_received_ = true;
    return;
};

void IgnitionPlatform::groundTruthCallback(geometry_msgs::msg::Pose &pose_msg)
{
    if (!imu_info_received_)
    {
        RCLCPP_WARN(rclcpp::get_logger("as2_ignition_platform"), "IMU not received yet");
        return;
    }

    geometry_msgs::msg::Pose last_pose;
    geometry_msgs::msg::Twist twist_msg_enu;

    // Derive twist from ground truth
    static auto last_time = rclcpp::Clock().now();
    auto current_time = rclcpp::Clock().now();
    auto dt = (current_time - last_time).seconds();
    last_time = current_time;

    static auto last_pose_msg = pose_msg;
    geometry_msgs::msg::Pose delta_pose;
    delta_pose.position.x = pose_msg.position.x - last_pose_msg.position.x;
    delta_pose.position.y = pose_msg.position.y - last_pose_msg.position.y;
    delta_pose.position.z = pose_msg.position.z - last_pose_msg.position.z;
    last_pose_msg = pose_msg;

    static auto last_vx = delta_pose.position.x / dt;
    static auto last_vy = delta_pose.position.y / dt;
    static auto last_vz = delta_pose.position.z / dt;

    const double alpha = 0.1f;

    twist_msg_enu.linear.x = alpha * (delta_pose.position.x / dt) + (1 - alpha) * last_vx;
    twist_msg_enu.linear.y = alpha * (delta_pose.position.y / dt) + (1 - alpha) * last_vy;
    twist_msg_enu.linear.z = alpha * (delta_pose.position.z / dt) + (1 - alpha) * last_vz;

    last_vx = twist_msg_enu.linear.x;
    last_vy = twist_msg_enu.linear.y;
    last_vz = twist_msg_enu.linear.z;

    // Set angular velocity in ENU frame
    twist_msg_enu.angular = imu_msg_->angular_velocity;

    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = generateTfName(namespace_, "map");
    pose_stamped_msg.header.stamp = rclcpp::Clock().now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_pub_->publish(pose_stamped_msg);

    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    twist_stamped_msg.header.frame_id = generateTfName(namespace_, "map");
    twist_stamped_msg.header.stamp = rclcpp::Clock().now();
    twist_stamped_msg.twist = twist_msg_enu;
    ground_truth_twist_pub_->publish(twist_stamped_msg);

    RCLCPP_INFO_ONCE(rclcpp::get_logger("as2_ignition_platform"), "Published ground truth");

    self_orientation_ = pose_msg.orientation;
    odometry_info_received_ = true;
    return;
};

void IgnitionPlatform::imuCallback(const sensor_msgs::msg::Imu &msg)
{
    if (!imu_info_received_)
    {
        imu_msg_ = std::make_unique<sensor_msgs::msg::Imu>(msg);
        imu_info_received_ = true;
    }
    imu_msg_->angular_velocity = msg.angular_velocity;
    imu_msg_->linear_acceleration = msg.linear_acceleration;
    imu_msg_->orientation = msg.orientation;
    imu_msg_->header.stamp = msg.header.stamp;
    imu_msg_->header.frame_id = msg.header.frame_id;
    return;
}

void IgnitionPlatform::usvCallback(geometry_msgs::msg::Pose &pose_msg)
{
    geometry_msgs::msg::Pose last_pose;
    geometry_msgs::msg::Twist twist_msg_enu;

    // Derive twist from ground truth
    static auto last_time_usv = rclcpp::Clock().now();
    auto current_time = rclcpp::Clock().now();
    auto dt = (current_time - last_time_usv).seconds();
    last_time_usv = current_time;

    static auto last_pose_msg_usv = pose_msg;
    geometry_msgs::msg::Pose delta_pose;
    delta_pose.position.x = pose_msg.position.x - last_pose_msg_usv.position.x;
    delta_pose.position.y = pose_msg.position.y - last_pose_msg_usv.position.y;
    delta_pose.position.z = pose_msg.position.z - last_pose_msg_usv.position.z;
    last_pose_msg_usv = pose_msg;

    static auto last_vx_usv = delta_pose.position.x / dt;
    static auto last_vy_usv = delta_pose.position.y / dt;
    static auto last_vz_usv = delta_pose.position.z / dt;

    const double alpha = 0.1f;

    twist_msg_enu.linear.x = alpha * (delta_pose.position.x / dt) + (1 - alpha) * last_vx_usv;
    twist_msg_enu.linear.y = alpha * (delta_pose.position.y / dt) + (1 - alpha) * last_vy_usv;
    twist_msg_enu.linear.z = alpha * (delta_pose.position.z / dt) + (1 - alpha) * last_vz_usv;

    last_vx_usv = twist_msg_enu.linear.x;
    last_vy_usv = twist_msg_enu.linear.y;
    last_vz_usv = twist_msg_enu.linear.z;

    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = "earth";
    pose_stamped_msg.header.stamp = rclcpp::Clock().now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_usv_pub_->publish(pose_stamped_msg);

    geometry_msgs::msg::TwistStamped twist_stamped_msg;
    twist_stamped_msg.header.frame_id = "earth";
    twist_stamped_msg.header.stamp = rclcpp::Clock().now();
    twist_stamped_msg.twist = twist_msg_enu;
    ground_truth_twist_usv_pub_->publish(twist_stamped_msg);

    return;
}

void IgnitionPlatform::targetACallback(geometry_msgs::msg::Pose &pose_msg)
{
    RCLCPP_INFO_ONCE(rclcpp::get_logger("as2_ignition_platform"), "Target A");
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = "earth";
    pose_stamped_msg.header.stamp = IgnitionPlatform::clock_->now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_targetA_pub_->publish(pose_stamped_msg);

    return;
}

void IgnitionPlatform::targetBCallback(geometry_msgs::msg::Pose &pose_msg)
{
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = "earth";
    pose_stamped_msg.header.stamp = rclcpp::Clock().now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_targetB_pub_->publish(pose_stamped_msg);

    return;
}

void IgnitionPlatform::targetCCallback(geometry_msgs::msg::Pose &pose_msg)
{
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = "earth";
    pose_stamped_msg.header.stamp = rclcpp::Clock().now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_targetC_pub_->publish(pose_stamped_msg);

    return;
}

void IgnitionPlatform::targetDCallback(geometry_msgs::msg::Pose &pose_msg)
{
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.frame_id = "earth";
    pose_stamped_msg.header.stamp = rclcpp::Clock().now();
    pose_stamped_msg.pose = pose_msg;
    ground_truth_pose_targetD_pub_->publish(pose_stamped_msg);

    return;
}

void IgnitionPlatform::poseCallback(const geometry_msgs::msg::PoseStamped &pose_msg)
{
    self_orientation_ = pose_msg.pose.orientation;
    odometry_info_received_ = true;
    return;
}

} // namespace ignition_platform
