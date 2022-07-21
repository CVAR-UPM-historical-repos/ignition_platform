/*!*******************************************************************************************
 *  \file       ignition_bridge.cpp
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

#include "ignition_bridge.hpp"

namespace ignition_platform
{

  std::string IgnitionBridge::name_space_ = "";
  odometryCallbackType IgnitionBridge::odometryCallback_ = [](nav_msgs::msg::Odometry &msg) {};
  groundTruthCallbackType IgnitionBridge::groundTruthCallback_ = [](geometry_msgs::msg::Pose &msg) {};

  IgnitionBridge::IgnitionBridge(std::string name_space, std::string world_name)
  {
    name_space_ = name_space;

    // Initialize the ignition node
    ign_node_ptr_ = std::make_shared<ignition::transport::Node>();

    // Initialize subscribers
    ign_node_ptr_->Subscribe("/model" + name_space + "/odometry",
                             IgnitionBridge::ignitionOdometryCallback);

    RCLCPP_INFO(rclcpp::get_logger("ignition_bridge"), "Suscribe to model%s/odometry", name_space.c_str());

    ign_node_ptr_->Subscribe("/world/" + world_name + "/pose/info",
                             IgnitionBridge::ignitionGroundTruthCallback);
    RCLCPP_INFO(rclcpp::get_logger("ignition_bridge"), "Suscribe to /world/%s/pose/info", world_name.c_str());

    return;
  };

  void IgnitionBridge::setOdometryCallback(odometryCallbackType callback)
  {
    odometryCallback_ = callback;
    return;
  };

  void IgnitionBridge::ignitionOdometryCallback(const ignition::msgs::Odometry &msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("ignition_bridge"), "Send odometry message");
    nav_msgs::msg::Odometry odom_msg;
    ros_ign_bridge::convert_ign_to_ros(msg, odom_msg);
    odometryCallback_(odom_msg);
    return;
  };

  void IgnitionBridge::setGroundTruthCallback(groundTruthCallbackType callback)
  {
    groundTruthCallback_ = callback;
    return;
  };

  void IgnitionBridge::ignitionGroundTruthCallback(const ignition::msgs::Pose_V &msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("ignition_bridge"), "Send ground truth pose");
    // Remove firts element of name_space_
    std::string name_space_2 = name_space_.substr(1);
    for (auto const &p : msg.pose())
    {
      RCLCPP_INFO(rclcpp::get_logger("ignition_bridge"), "Pose name: %s", p.name().c_str());
      if (p.name() == name_space_2)
      {
        geometry_msgs::msg::Pose pose;
        ros_ign_bridge::convert_ign_to_ros(p, pose);
        groundTruthCallback_(pose);
        return;
      }
    }
    RCLCPP_WARN(rclcpp::get_logger("ignition_bridge"), "No pose found for %s", name_space_2.c_str());
    return;
  };

} // namespace ignition_platform
