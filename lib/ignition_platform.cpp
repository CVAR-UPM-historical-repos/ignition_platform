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
    std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> IgnitionPlatform::pose_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Camera> IgnitionPlatform::camera_ptr_ = nullptr;
    bool IgnitionPlatform::camera_info_received_ = false;
    bool IgnitionPlatform::odometry_info_received_ = false;
    double IgnitionPlatform::yaw_ = 0.0;

    IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
    {
        ignition_bridge_ = std::make_shared<IgnitionBridge>(this->get_namespace());
        this->configureSensors();

        this->declare_parameter("sensors");
        static auto parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&IgnitionPlatform::parametersCallback, this, std::placeholders::_1));

        // Timer to send command
        static auto timer_commands_ =
            this->create_wall_timer(std::chrono::milliseconds(CMD_FREQ), [this]()
                                    { this->sendCommand(); });
    };

    void IgnitionPlatform::configureSensors()
    {
        pose_ptr_ =
            std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>("pose", this);
        ignition_bridge_->setPoseCallback(poseCallback);

        odometry_raw_estimation_ptr_ =
            std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
        ignition_bridge_->setOdometryCallback(odometryCallback);

        // IgnitionPlatform::camera_info_received_ = false;
        // camera_ptr_ =
        //     std::make_unique<as2::sensors::Camera>("image", this);
        // ignition_bridge_->setCameraCallback(cameraCallback);
        // ignition_bridge_->setCameraInfoCallback(cameraInfoCallback);

        return;
    };

    bool IgnitionPlatform::ownSendCommand()
    {
        if (control_in_.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME)
        {
            if (!odometry_info_received_)
            {
                return true;
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
            Eigen::Vector3d twist_lineal_flu = convertENUtoFLU(yaw_, twist_lineal_enu);
            command_twist_msg_.twist.linear.x = twist_lineal_flu(0);
            command_twist_msg_.twist.linear.y = twist_lineal_flu(1);
            command_twist_msg_.twist.linear.z = twist_lineal_flu(2);
            ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
        }
        else if (control_in_.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME)
        {
            ignition_bridge_->sendTwistMsg(command_twist_msg_.twist);
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

    std::vector<std::string> split(const std::string &s, char delim)
    {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim))
        {
            elems.push_back(item);
        }
        return elems;
    };

    rcl_interfaces::msg::SetParametersResult IgnitionPlatform::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (auto &param : parameters)
        {
            if (param.get_name() == "sensors")
            {
                std::string sensors_str = param.get_value<std::string>();
                std::vector<std::string> sensors_str_vec = split(sensors_str, ',');
                // TODO: process sensors_str_vec
                // for (auto &sensor_str : sensors_str_vec)
                // {
                //     if (sensor_str == "image")
                //     {
                //         camera_ptr_->set_active(true);
                //     }
                //     else
                //     {
                //         RCLCPP_WARN(this->get_logger(), "IgnitionPlatform::parametersCallback() - unknown sensor: %s", sensor_str.c_str());
                //     }
                // }
                
            }
        }
        return result;
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

        tf2::Quaternion q(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw_ = yaw;

        odometry_info_received_ = true;
        return;
    };

    void IgnitionPlatform::cameraCallback(const sensor_msgs::msg::Image &image_msg)
    {
        camera_ptr_->updateData(image_msg);
        return;
    };

    void IgnitionPlatform::cameraInfoCallback(const sensor_msgs::msg::CameraInfo &info_msg)
    {
        if (IgnitionPlatform::camera_info_received_)
        {
            return;
        }
        // camera_ptr_->setParameters(info_msg);
        IgnitionPlatform::camera_info_received_ = true;
        // camera_info_ptr_->updateData(info_msg);
        return;
    };

    // Convert from ENU (east, north, up) to local FLU (forward, left, up)
    Eigen::Vector3d IgnitionPlatform::convertENUtoFLU(
        const float yaw_angle,
        Eigen::Vector3d &enu_vec)
    {
        // Convert from ENU to FLU
        Eigen::Matrix3d R_FLU;
        R_FLU << cos(yaw_angle), sin(yaw_angle), 0,
            -sin(yaw_angle), cos(yaw_angle), 0,
            0, 0, 1;

        return R_FLU * enu_vec;
    }
}