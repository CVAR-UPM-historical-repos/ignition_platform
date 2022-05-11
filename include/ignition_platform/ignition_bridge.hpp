#ifndef IGNITION_BRIDGE_HPP_
#define IGNITION_BRIDGE_HPP_

#include <memory>
#include <string>
#include <iostream>

#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ros_ign_bridge/convert.hpp>

#include <string>

namespace ignition_platform
{
    typedef void (*poseCallbackType)(const geometry_msgs::msg::PoseStamped &msg);
    typedef void (*odometryCallbackType)(const nav_msgs::msg::Odometry &msg);
    typedef void (*cameraCallbackType)(const sensor_msgs::msg::Image &msg);
    typedef void (*cameraInfoCallbackType)(const sensor_msgs::msg::CameraInfo &msg);

    class IgnitionBridge
    {
    public:
        IgnitionBridge(std::string name_space = "/");
        ~IgnitionBridge(){};

    public:
        std::shared_ptr<ignition::transport::Node> ign_node_ptr_;

        ignition::msgs::Twist ign_msg_;
        ignition::transport::v11::Node::Publisher command_twist_pub_;
        
    private:
        const std::string &ign_topic_command_twist_ = "/cmd_vel";

        const std::string &ign_topic_sensor_pose_ = "/pose";
        const std::string &ign_topic_sensor_odometry_ = "/odometry";
        const std::string &ign_topic_sensor_camera_ = "/camera";
        const std::string &ign_topic_sensor_camera_info_ = "/camera_info";

    public:
        void sendTwistMsg(const geometry_msgs::msg::Twist &msg);
        void setPoseCallback(poseCallbackType callback);
        void setOdometryCallback(odometryCallbackType callback);
        void setCameraCallback(cameraCallbackType callback);
        void setCameraInfoCallback(cameraInfoCallbackType callback);

    private:
        static poseCallbackType poseCallback_;
        static void ignitionPoseCallback(const ignition::msgs::Pose &msg);

        static odometryCallbackType odometryCallback_;
        static void ignitionOdometryCallback(const ignition::msgs::Odometry &msg);

        static cameraCallbackType cameraCallback_;
        static void ignitionCameraCallback(const ignition::msgs::Image &msg);

        static cameraInfoCallbackType cameraInfoCallback_;
        static void ignitionCameraInfoCallback(const ignition::msgs::CameraInfo &msg);
        
    };
}

#endif // IGNITION_BRIDGE_HPP_