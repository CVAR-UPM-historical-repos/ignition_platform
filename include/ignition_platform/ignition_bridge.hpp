/*!*******************************************************************************************
 *  \file       ignition_bridge.hpp
 *  \brief      Implementation of an Ignition Gazebo bridge to ROS
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

#ifndef IGNITION_BRIDGE_HPP_
#define IGNITION_BRIDGE_HPP_

#include <memory>
#include <string>
#include <iostream>

#include <unordered_map>
#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <tf2_msgs/msg/tf_message.h>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ros_ign_bridge/convert.hpp>

namespace ignition_platform
{
    typedef void (*poseCallbackType)(geometry_msgs::msg::PoseStamped &msg);
    typedef void (*odometryCallbackType)(nav_msgs::msg::Odometry &msg);

    typedef void (*imuSensorCallbackType)(sensor_msgs::msg::Imu &msg);
    typedef void (*airPressureSensorCallbackType)(sensor_msgs::msg::FluidPressure &msg);
    typedef void (*magnetometerSensorCallbackType)(sensor_msgs::msg::MagneticField &msg);

    typedef void (*tfCallbackType)(geometry_msgs::msg::TransformStamped &msg, const std::string &sensor_name);
    typedef void (*cameraCallbackType)(sensor_msgs::msg::Image &msg, const std::string &sensor_name);
    typedef void (*cameraInfoCallbackType)(sensor_msgs::msg::CameraInfo &msg, const std::string &sensor_name);
    typedef void (*laserScanCallbackType)(sensor_msgs::msg::LaserScan &msg, const std::string &sensor_name);
    typedef void (*pointCloudCallbackType)(sensor_msgs::msg::PointCloud2 &msg, const std::string &sensor_name);
    typedef void (*gpsCallbackType)(sensor_msgs::msg::NavSatFix &msg, const std::string &sensor_name);
    typedef void (*imuCallbackType)(sensor_msgs::msg::Imu &msg, const std::string &sensor_name);
    typedef void (*airPressureCallbackType)(sensor_msgs::msg::FluidPressure &msg, const std::string &sensor_name);
    typedef void (*magnetometerCallbackType)(sensor_msgs::msg::MagneticField &msg, const std::string &sensor_name);

    class IgnitionBridge
    {
    public:
        IgnitionBridge(std::string name_space = "/");
        ~IgnitionBridge(){};

    public:
        std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
        ignition::transport::v11::Node::Publisher command_twist_pub_;

    private:
        static std::string name_space_;

        const std::string &ign_topic_command_twist_ = "/cmd_vel";
        const std::string &ign_topic_sensor_pose_ = "/pose";
        const std::string &ign_topic_sensor_pose_static_ = "/pose_static";
        const std::string &ign_topic_sensor_odometry_ = "/odometry";

    public:
        void sendTwistMsg(const geometry_msgs::msg::Twist &msg);

        void unsuscribePoseStatic();

        void setPoseCallback(poseCallbackType callback);
        void setOdometryCallback(odometryCallbackType callback);

        void setImuCallback(imuSensorCallbackType callback, std::string world_name);
        void setAirPressureCallback(airPressureSensorCallbackType callback, std::string world_name);
        void setMagnetometerCallback(magnetometerSensorCallbackType callback, std::string world_name);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            cameraCallbackType cameraCallback,
            cameraInfoCallbackType cameraInfoCallback,
            tfCallbackType tfCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            laserScanCallbackType laserScanCallback,
            pointCloudCallbackType pointCloudCallback,
            tfCallbackType tfCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            gpsCallbackType gpsCallback,
            tfCallbackType tfCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            imuCallbackType imuCallback,
            tfCallbackType tfCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            airPressureCallbackType air_pressureCallback,
            tfCallbackType tfCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            magnetometerCallbackType magnetometerCallback,
            tfCallbackType tfCallback);

    private:
        // Ignition callbacks
        static poseCallbackType poseCallback_;
        static void ignitionPoseCallback(const ignition::msgs::Pose &msg);
        static odometryCallbackType odometryCallback_;
        static void ignitionOdometryCallback(const ignition::msgs::Odometry &msg);

        static imuSensorCallbackType imuCallback_;
        static void ignitionImuSensorCallback(const ignition::msgs::IMU &msg);
        static airPressureSensorCallbackType airPressureCallback_;
        static void ignitionAirPressureSensorCallback(const ignition::msgs::FluidPressure &msg);
        static magnetometerSensorCallbackType magnetometerCallback_;
        static void ignitionMagnetometerSensorCallback(const ignition::msgs::Magnetometer &msg);

        static void ignitionPoseStaticCallback(const ignition::msgs::Pose_V &msg);

        static std::unordered_map<std::string, std::string> callbacks_sensors_names_;
        static std::unordered_map<std::string, tfCallbackType> callbacks_sensors_transform_;

        static void ignitionCameraCallback(const ignition::msgs::Image &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, cameraCallbackType> callbacks_camera_;
        static void ignitionCameraInfoCallback(const ignition::msgs::CameraInfo &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, cameraInfoCallbackType> callbacks_camera_info_;

        static void ignitionLaserScanCallback(const ignition::msgs::LaserScan &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, laserScanCallbackType> callbacks_laser_scan_;
        static void ignitionPointCloudCallback(const ignition::msgs::PointCloudPacked &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, pointCloudCallbackType> callbacks_point_cloud_;

        static void ignitionGPSCallback(const ignition::msgs::NavSat &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, gpsCallbackType> callbacks_gps_;

        static void ignitionImuCallback(const ignition::msgs::IMU &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, imuCallbackType> callbacks_imu_;

        static void ignitionAirPressureCallback(const ignition::msgs::FluidPressure &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, airPressureCallbackType> callbacks_air_pressure_;

        static void ignitionMagnometerCallback(const ignition::msgs::Magnetometer &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, magnetometerCallbackType> callbacks_magnetometer_;
    };
}

#endif // IGNITION_BRIDGE_HPP_