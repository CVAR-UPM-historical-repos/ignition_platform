/*!*******************************************************************************************
 *  \file       ignition_platform.hpp
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

#ifndef IGNITION_PLATFORM_HPP_
#define IGNITION_PLATFORM_HPP_

#include <memory>
#include <string>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>

#include <math.h>
#include <as2_core/core_functions.hpp>
#include <as2_core/aerial_platform.hpp>
#include <as2_core/frame_utils/frame_utils.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


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
        void configureSensors(){};
        bool ownSendCommand() override;
        bool ownSetArmingState(bool state) override;
        bool ownSetOffboardControl(bool offboard) override;
        bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) override;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        void poseCallback(const geometry_msgs::msg::PoseStamped &msg);

    private:
        as2_msgs::msg::ControlMode control_in_;
        bool odometry_info_received_ = false;
        geometry_msgs::msg::Quaternion self_orientation_;
        double yaw_rate_limit_ = M_PI_2;

    private:
        void resetCommandTwistMsg();
    };
}

#endif // IGNITION_PLATFORM_HPP_
