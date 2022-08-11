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
#include <as2_core/frame_utils/frame_utils.hpp>
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
    typedef void (*odometryCallbackType)(nav_msgs::msg::Odometry &msg);
    typedef void (*groundTruthCallbackType)(geometry_msgs::msg::Pose &msg);
    
    class IgnitionBridge
    {
    public:
        IgnitionBridge(std::string name_space = "/", std::string world_name = "");
        ~IgnitionBridge(){};

    public:
        std::shared_ptr<ignition::transport::Node> ign_node_ptr_;

    private:
        static std::string name_space_;

    public:
        void setOdometryCallback(odometryCallbackType callback);
        void setGroundTruthCallback(groundTruthCallbackType callback);

        void setUSVCallback(groundTruthCallbackType callback);
        static groundTruthCallbackType usvCallback_;
        void setTargetACallback(groundTruthCallbackType callback);
        static groundTruthCallbackType targetACallback_;

    private:
        // Ignition callbacks
        static odometryCallbackType odometryCallback_;
        static void ignitionOdometryCallback(const ignition::msgs::Odometry &msg);
        static groundTruthCallbackType groundTruthCallback_;
        static void ignitionGroundTruthCallback(const ignition::msgs::Pose_V &msg);

    public:
        ignition::transport::v11::Node::Publisher command_twist_pub_;
        void sendTwistMsg(const geometry_msgs::msg::Twist &msg);

    };
}

#endif // IGNITION_BRIDGE_HPP_
