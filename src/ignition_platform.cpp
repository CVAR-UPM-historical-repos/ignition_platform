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

namespace ignition_platform {
IgnitionPlatform::IgnitionPlatform() : as2::AerialPlatform() {
  this->declare_parameter<std::string>("cmd_vel_topic");
  std::string cmd_vel_topic_param = this->get_parameter("cmd_vel_topic").as_string();

  this->declare_parameter<std::string>("arm_topic");
  std::string arm_topic_param = this->get_parameter("arm_topic").as_string();

  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->generate_global_name(cmd_vel_topic_param), rclcpp::QoS(1));

  arm_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->generate_global_name(arm_topic_param), rclcpp::QoS(1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos,
      std::bind(&IgnitionPlatform::poseCallback, this, std::placeholders::_1));
};

bool IgnitionPlatform::ownSendCommand() {
  if (control_in_.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME) {
    if (!odometry_info_received_) {
      return false;
    }
    odometry_info_received_ = false;

    if (command_twist_msg_.twist.angular.z > yaw_rate_limit_) {
      command_twist_msg_.twist.angular.z = yaw_rate_limit_;
    } else if (command_twist_msg_.twist.angular.z < -yaw_rate_limit_) {
      command_twist_msg_.twist.angular.z = -yaw_rate_limit_;
    }

    Eigen::Vector3d twist_lineal_enu =
        Eigen::Vector3d(command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y,
                        command_twist_msg_.twist.linear.z);

    Eigen::Vector3d twist_lineal_flu =
        as2::frame::convertENUtoFLU(self_orientation_, twist_lineal_enu);
    command_twist_msg_.twist.linear.x = twist_lineal_flu(0);
    command_twist_msg_.twist.linear.y = twist_lineal_flu(1);
    command_twist_msg_.twist.linear.z = twist_lineal_flu(2);

    twist_pub_->publish(command_twist_msg_.twist);
    
  } else if (control_in_.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME) {
    twist_pub_->publish(command_twist_msg_.twist);
  }
  return true;
};

bool IgnitionPlatform::ownSetArmingState(bool state) {
  std_msgs::msg::Bool arm_msg;
  arm_msg.data = state;
  arm_pub_->publish(arm_msg);
  resetCommandTwistMsg();
  return true;
};

bool IgnitionPlatform::ownSetOffboardControl(bool offboard) {
  return true;
};

bool IgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in) {
  control_in_ = control_in;
  return true;
};

void IgnitionPlatform::resetCommandTwistMsg() {
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x  = 0.0f;
  twist_msg.linear.y  = 0.0f;
  twist_msg.linear.z  = 0.0f;
  twist_msg.angular.x = 0.0f;
  twist_msg.angular.y = 0.0f;
  twist_msg.angular.z = 0.0f;
  twist_pub_->publish(twist_msg);
}

void IgnitionPlatform::poseCallback(const geometry_msgs::msg::PoseStamped &pose_msg) {
  self_orientation_       = pose_msg.pose.orientation;
  odometry_info_received_ = true;
  return;
}
}  // namespace ignition_platform
