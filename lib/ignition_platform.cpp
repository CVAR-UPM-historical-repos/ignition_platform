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
    bool IgnitionPlatform::odometry_info_received_ = false;
    geometry_msgs::msg::Quaternion IgnitionPlatform::self_orientation_ = geometry_msgs::msg::Quaternion();
    std::string IgnitionPlatform::namespace_ = "";

    std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> IgnitionPlatform::pose_ptr_ = nullptr;
    std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> IgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

    std::unordered_map<std::string, as2::sensors::Camera> IgnitionPlatform::callbacks_camera_ = {};
    std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> IgnitionPlatform::callbacks_laser_scan_ = {};
    std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::PointCloud2>> IgnitionPlatform::callbacks_point_cloud_ = {};

    std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::NavSatFix>> IgnitionPlatform::callbacks_gps_ = {};

    IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform()
    {
        this->declare_parameter("sensors");
        namespace_ = this->get_namespace();
        ignition_bridge_ = std::make_shared<IgnitionBridge>(namespace_);

        this->configureSensors();

        // Timer to send command
        static auto timer_commands_ =
            this->create_wall_timer(
                std::chrono::milliseconds(CMD_FREQ),
                [this](){ this->sendCommand();});
    };

    std::vector<std::string> split(const std::string &s, char delim)
    {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim))
        {
            elems.emplace_back(item);
        }
        return elems;
    };

    void IgnitionPlatform::configureSensors()
    {
        pose_ptr_ =
            std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>("pose", this);
        ignition_bridge_->setPoseCallback(poseCallback);

        odometry_raw_estimation_ptr_ =
            std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(
                "odom", this);
        ignition_bridge_->setOdometryCallback(odometryCallback);

        std::string sensors_param = this->get_parameter("sensors").as_string();
        std::vector<std::string> sensor_config_list = split(sensors_param, ':');

        for (auto sensor_config : sensor_config_list)
        {
            std::vector<std::string> sensor_config_params = split(sensor_config, ',');

            if (sensor_config_params.size() != 5)
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Wrong sensor configuration: %s",
                                  sensor_config.c_str());
                continue;
            }

            std::string sensor_type = sensor_config_params[4];
            if (sensor_type == "camera")
            {
                as2::sensors::Camera camera = as2::sensors::Camera(sensor_config_params[2], this);
                callbacks_camera_.insert(std::make_pair(sensor_config_params[2],
                                                        camera));

                ignition_bridge_->addSensor(
                    sensor_config_params[0],
                    sensor_config_params[1],
                    sensor_config_params[2],
                    sensor_config_params[3],
                    sensor_config_params[4],
                    cameraCallback,
                    cameraInfoCallback);
            }
            else if (sensor_type == "lidar")
            {
                as2::sensors::Sensor<sensor_msgs::msg::LaserScan> laser_scan_sensor(sensor_config_params[2], this);
                as2::sensors::Sensor<sensor_msgs::msg::PointCloud2> point_cloud_sensor(sensor_config_params[2] + "/points", this);

                callbacks_laser_scan_.insert(std::make_pair(sensor_config_params[2], laser_scan_sensor));
                callbacks_point_cloud_.insert(std::make_pair(sensor_config_params[2], point_cloud_sensor));

                ignition_bridge_->addSensor(
                    sensor_config_params[0],
                    sensor_config_params[1],
                    sensor_config_params[2],
                    sensor_config_params[3],
                    sensor_config_params[4],
                    laserScanCallback,
                    pointCloudCallback);
            }
            else if (sensor_type == "gps")
            {
                as2::sensors::Sensor<sensor_msgs::msg::NavSatFix> gps_sensor(sensor_config_params[2], this);
                callbacks_gps_.insert(std::make_pair(sensor_config_params[2], gps_sensor));

                ignition_bridge_->addSensor(
                    sensor_config_params[0],
                    sensor_config_params[1],
                    sensor_config_params[2],
                    sensor_config_params[3],
                    sensor_config_params[4],
                    gpsCallback);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Sensor type not supported: %s", sensor_type.c_str());
            }
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
            
            Eigen::Vector3d twist_lineal_enu = Eigen::Vector3d(command_twist_msg_.twist.linear.x,
                                                               command_twist_msg_.twist.linear.y,
                                                               command_twist_msg_.twist.linear.z);

            Eigen::Vector3d twist_lineal_flu = as2::FrameUtils::convertENUtoFLU(self_orientation_, twist_lineal_enu);
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

    void IgnitionPlatform::poseCallback(geometry_msgs::msg::PoseStamped &pose_msg)
    {
        pose_ptr_->updateData(pose_msg);
        return;
    };

    void IgnitionPlatform::odometryCallback(nav_msgs::msg::Odometry &odom_msg)
    {
        odom_msg.header.frame_id = generateTfName(namespace_, "odom");

        odometry_raw_estimation_ptr_->updateData(odom_msg);

        self_orientation_ = odom_msg.pose.pose.orientation;
        odometry_info_received_ = true;
        return;
    };

    void IgnitionPlatform::cameraCallback(
        sensor_msgs::msg::Image &image_msg,
        const std::string &sensor_name)
    {
        (callbacks_camera_.find(sensor_name)->second).updateData(image_msg);
        return;
    };

    void IgnitionPlatform::cameraInfoCallback(
        sensor_msgs::msg::CameraInfo &info_msg,
        const std::string &sensor_name)
    {
        (callbacks_camera_.find(sensor_name)->second).setParameters(info_msg);
        return;
    };

    void IgnitionPlatform::laserScanCallback(
        sensor_msgs::msg::LaserScan &laser_scan_msg,
        const std::string &sensor_name)
    {
        laser_scan_msg.header.frame_id = "map";
        (callbacks_laser_scan_.find(sensor_name)->second).updateData(laser_scan_msg);
        return;
    };

    void IgnitionPlatform::pointCloudCallback(
        sensor_msgs::msg::PointCloud2 &point_cloud_msg,
        const std::string &sensor_name)
    {
        point_cloud_msg.header.frame_id = "map";
        (callbacks_point_cloud_.find(sensor_name)->second).updateData(point_cloud_msg);
        return;
    };

    void IgnitionPlatform::gpsCallback(
        sensor_msgs::msg::NavSatFix &gps_msg, 
        const std::string &sensor_name)
    {
        (callbacks_gps_.find(sensor_name)->second).updateData(gps_msg);
        return;
    };
}
