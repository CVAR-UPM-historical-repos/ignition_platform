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
    geometry_msgs::msg::Quaternion IgnitionPlatform::self_orientation_ = geometry_msgs::msg::Quaternion();
    std::string IgnitionPlatform::namespace_ = "";

    // std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;
    // std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::Pose>> IgnitionPlatform::ground_truth_ptr_ = nullptr;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IgnitionPlatform::ground_truth_pose_pub_ = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr IgnitionPlatform::ground_truth_twist_pub_ = nullptr;

    IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
    {
        this->declare_parameter<std::string>("world");
        std::string world_param = this->get_parameter("world").as_string();
        RCLCPP_INFO(this->get_logger(), "World: %s", world_param.c_str());

        namespace_ = this->get_namespace();
        
        ignition_bridge_ = std::make_shared<IgnitionBridge>(namespace_, world_param);

        this->configureSensors();

        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(namespace_ + "/cmd_vel",
                                                                       rclcpp::QoS(1));

        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        //     as2_names::topics::self_localization::odom,
        //     as2_names::topics::self_localization::qos);

        ground_truth_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            as2_names::topics::ground_truth::pose,
            as2_names::topics::ground_truth::qos);

        ground_truth_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            as2_names::topics::ground_truth::twist,
            as2_names::topics::ground_truth::qos);

        // Timer to send command
        static auto timer_commands_ =
            this->create_wall_timer(
                std::chrono::milliseconds(CMD_FREQ),
                [this]()
                { this->sendCommand(); });
    };

    void IgnitionPlatform::configureSensors()
    {
        // odometry_raw_estimation_ptr_ =
        //     std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(
        //         "odom", this);
        // ignition_bridge_->setOdometryCallback(odometryCallback);

        // ground_truth_ptr_ =
        //     std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::Pose>>(
        //         "ground_truth", this);
        ignition_bridge_->setGroundTruthCallback(groundTruthCallback);

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

            Eigen::Vector3d twist_lineal_enu = Eigen::Vector3d(command_twist_msg_.twist.linear.x,
                                                               command_twist_msg_.twist.linear.y,
                                                               command_twist_msg_.twist.linear.z);

            Eigen::Vector3d twist_lineal_flu = as2::FrameUtils::convertENUtoFLU(self_orientation_, twist_lineal_enu);
            command_twist_msg_.twist.linear.x = twist_lineal_flu(0);
            command_twist_msg_.twist.linear.y = twist_lineal_flu(1);
            command_twist_msg_.twist.linear.z = twist_lineal_flu(2);

            twist_pub_->publish(command_twist_msg_.twist);
        }
        else if (control_in_.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME)
        {   
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

        twist_pub_->publish(twist_msg);
    }

    // void IgnitionPlatform::odometryCallback(nav_msgs::msg::Odometry &odom_msg)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("ignition_platform"), "Odometry received");
    //     odom_msg.header.frame_id = generateTfName(namespace_, "odom");

    //     odometry_raw_estimation_ptr_->updateData(odom_msg);

    //     self_orientation_ = odom_msg.pose.pose.orientation;
    //     odometry_info_received_ = true;
    //     return;
    // };

    void IgnitionPlatform::groundTruthCallback(geometry_msgs::msg::Pose &pose_msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("ignition_platform"), "Ground truth received");
        geometry_msgs::msg::Pose last_pose;
        geometry_msgs::msg::Twist twist_msg_enu;
        // geometry_msgs::msg::Twist twist_msg_flu;

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

        twist_msg_enu.linear.x = delta_pose.position.x / dt;
        twist_msg_enu.linear.y = delta_pose.position.y / dt;
        twist_msg_enu.linear.z = delta_pose.position.z / dt;

        // Eigen::Vector3d twist_lineal_enu = Eigen::Vector3d(twist_msg_enu.linear.x,
        //                                                    twist_msg_enu.linear.y,
        //                                                    twist_msg_enu.linear.z);

        // Eigen::Vector3d twist_lineal_flu = as2::FrameUtils::convertENUtoFLU(pose_msg.orientation, twist_lineal_enu);
        // twist_msg_flu.linear.x = twist_lineal_flu(0);
        // twist_msg_flu.linear.y = twist_lineal_flu(1);
        // twist_msg_flu.linear.z = twist_lineal_flu(2);

        tf2::Quaternion tf_quaternion(
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w);
        tf2::Matrix3x3 rotation_matrix(tf_quaternion);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        tf2::Quaternion tf_last_quaternion(
                last_pose_msg.orientation.x,
                last_pose_msg.orientation.y,
                last_pose_msg.orientation.z,
                last_pose_msg.orientation.w);
        tf2::Matrix3x3 last_rotation_matrix(tf_last_quaternion);
        double last_roll, last_pitch, last_yaw;
        last_rotation_matrix.getRPY(last_roll, last_pitch, last_yaw);
        
        double delta_pitch = pitch - last_pitch;
        if (delta_pitch > M_PI)
        {
            delta_pitch -= 2 * M_PI;
        }
        else if (delta_pitch < -M_PI)
        {
            delta_pitch += 2 * M_PI;
        }
        // twist_msg_flu.angular.x = delta_pitch / dt;
        twist_msg_enu.angular.x = delta_pitch / dt;

        double delta_roll = roll - last_roll;
        if (delta_roll > M_PI)
        {
            delta_roll -= 2 * M_PI;
        }
        else if (delta_roll < -M_PI)
        {
            delta_roll += 2 * M_PI;
        }
        // twist_msg_flu.angular.y = delta_roll / dt;
        twist_msg_enu.angular.y = delta_roll / dt;

        double delta_yaw = yaw - last_yaw;
        if (delta_yaw > M_PI)
        {
            delta_yaw -= 2 * M_PI;
        }
        else if (delta_yaw < -M_PI)
        {
            delta_yaw += 2 * M_PI;
        }
        // twist_msg_flu.angular.z = delta_yaw / dt;
        twist_msg_enu.angular.z = delta_yaw / dt;

        // nav_msgs::msg::Odometry odom_msg;
        // odom_msg.header.frame_id = generateTfName(namespace_, "map");
        // odom_msg.header.stamp = rclcpp::Clock().now();
        // odom_msg.pose.pose = pose_msg;
        // odom_msg.twist.twist = twist_msg_enu;
        // ground_truth_ptr_->updateData(pose_msg);

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

        odometry_info_received_ = true;
        return;
    };

}
