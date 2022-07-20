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

namespace ignition_platform {
static std::string refactorizeFrameId(const std::string &original_frame_id,
                                      const std::string &sensor_name,
                                      const std::string &field_to_append = "") {
  // check if there is a namespace in the frame_id it must be before the sensor_name
  std::string frame_id = original_frame_id;
  std::string ns = "";
  if (frame_id.find("base_link") != std::string::npos) {
    ns = frame_id.substr(0, frame_id.find("base_link/"));
    // remove / from the end of the namespace
    if (ns.back() == '/') {
      ns.pop_back();
    }
  } else if (frame_id.find(sensor_name) != std::string::npos) {
    ns = frame_id.substr(0, frame_id.find(sensor_name));
    // remove / from the end of the namespace
    if (ns.back() == '/') {
      ns.pop_back();
    }
  }
  if (field_to_append != "") {
    frame_id = ns + "/" + sensor_name + "/" + field_to_append;
  } else {
    frame_id = ns + "/" + sensor_name;
  }
  return frame_id;
};

std::string IgnitionBridge::name_space_ = "";
poseCallbackType IgnitionBridge::poseCallback_ = [](geometry_msgs::msg::PoseStamped &msg) {};
odometryCallbackType IgnitionBridge::odometryCallback_ = [](nav_msgs::msg::Odometry &msg) {};

imuSensorCallbackType IgnitionBridge::imuCallback_ = [](sensor_msgs::msg::Imu &msg) {};
airPressureSensorCallbackType IgnitionBridge::airPressureCallback_ =
    [](sensor_msgs::msg::FluidPressure &msg) {};
magnetometerSensorCallbackType IgnitionBridge::magnetometerCallback_ =
    [](sensor_msgs::msg::MagneticField &msg) {};

std::unordered_map<std::string, std::string> IgnitionBridge::callbacks_sensors_names_ = {};
std::unordered_map<std::string, tfCallbackType> IgnitionBridge::callbacks_sensors_transform_ = {};

std::unordered_map<std::string, cameraCallbackType> IgnitionBridge::callbacks_camera_ = {};
std::unordered_map<std::string, cameraInfoCallbackType> IgnitionBridge::callbacks_camera_info_ = {};
std::unordered_map<std::string, laserScanCallbackType> IgnitionBridge::callbacks_laser_scan_ = {};
std::unordered_map<std::string, pointCloudCallbackType> IgnitionBridge::callbacks_point_cloud_ = {};
std::unordered_map<std::string, gpsCallbackType> IgnitionBridge::callbacks_gps_ = {};
std::unordered_map<std::string, imuCallbackType> IgnitionBridge::callbacks_imu_ = {};
std::unordered_map<std::string, airPressureCallbackType> IgnitionBridge::callbacks_air_pressure_ =
    {};
std::unordered_map<std::string, magnetometerCallbackType> IgnitionBridge::callbacks_magnetometer_ =
    {};

IgnitionBridge::IgnitionBridge(std::string name_space) {
  name_space_ = name_space;

  // Initialize the ignition node
  ign_node_ptr_ = std::make_shared<ignition::transport::Node>();

  // Initialize publishers
  command_twist_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Twist>("model" + name_space +
                                                                       ign_topic_command_twist_);

  // Initialize subscribers
  ign_node_ptr_->Subscribe("model" + name_space + ign_topic_sensor_pose_,
                           IgnitionBridge::ignitionPoseCallback);

  ign_node_ptr_->Subscribe("model" + name_space + ign_topic_sensor_odometry_,
                           IgnitionBridge::ignitionOdometryCallback);

  ign_node_ptr_->Subscribe("model" + name_space + ign_topic_sensor_pose_static_,
                           IgnitionBridge::ignitionPoseStaticCallback);

  return;
};

void IgnitionBridge::sendTwistMsg(const geometry_msgs::msg::Twist &ros_twist_msg) {
  ignition::msgs::Twist ign_twist_msg;
  ros_ign_bridge::convert_ros_to_ign(ros_twist_msg, ign_twist_msg);
  command_twist_pub_.Publish(ign_twist_msg);
  return;
};

void IgnitionBridge::setPoseCallback(poseCallbackType callback) {
  poseCallback_ = callback;
  return;
};

void IgnitionBridge::ignitionPoseCallback(const ignition::msgs::Pose &msg) {
  geometry_msgs::msg::PoseStamped pose_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, pose_msg);
  poseCallback_(pose_msg);
  return;
};

void IgnitionBridge::setOdometryCallback(odometryCallbackType callback) {
  odometryCallback_ = callback;
  return;
};

void IgnitionBridge::ignitionOdometryCallback(const ignition::msgs::Odometry &msg) {
  nav_msgs::msg::Odometry odom_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, odom_msg);
  odometryCallback_(odom_msg);
  return;
};

void IgnitionBridge::setImuCallback(imuSensorCallbackType callback, std::string world_name) {
  imuCallback_ = callback;
  std::string ign_topic_sensor_imu = "/imu_sensor/imu";
  std::string topic = "world/" + world_name + "/model" + name_space_ + "/link/base_link/sensor" +
                      ign_topic_sensor_imu;
  ign_node_ptr_->Subscribe(topic, IgnitionBridge::ignitionImuSensorCallback);
  return;
};

void IgnitionBridge::ignitionImuSensorCallback(const ignition::msgs::IMU &msg) {
  sensor_msgs::msg::Imu imu_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, imu_msg);
  static auto frame_id = refactorizeFrameId(imu_msg.header.frame_id, "imu");
  imu_msg.header.frame_id = frame_id;
  imuCallback_(imu_msg);
  return;
};

void IgnitionBridge::setAirPressureCallback(airPressureSensorCallbackType callback,
                                            std::string world_name) {
  airPressureCallback_ = callback;
  std::string ign_topic_sensor_air_pressure = "/air_pressure/air_pressure";
  std::string topic = "world/" + world_name + "/model" + name_space_ + "/link/base_link/sensor" +
                      ign_topic_sensor_air_pressure;
  ign_node_ptr_->Subscribe(topic, IgnitionBridge::ignitionAirPressureSensorCallback);
  return;
};

void IgnitionBridge::ignitionAirPressureSensorCallback(const ignition::msgs::FluidPressure &msg) {
  sensor_msgs::msg::FluidPressure air_pressure_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, air_pressure_msg);
  airPressureCallback_(air_pressure_msg);
  return;
};

void IgnitionBridge::setMagnetometerCallback(magnetometerSensorCallbackType callback,
                                             std::string world_name) {
  magnetometerCallback_ = callback;
  std::string ign_topic_sensor_magnetometer = "/magnetometer/magnetometer";
  std::string topic = "world/" + world_name + "/model" + name_space_ + "/link/base_link/sensor" +
                      ign_topic_sensor_magnetometer;
  ign_node_ptr_->Subscribe(topic, IgnitionBridge::ignitionMagnetometerSensorCallback);
  return;
};

void IgnitionBridge::ignitionMagnetometerSensorCallback(const ignition::msgs::Magnetometer &msg) {
  sensor_msgs::msg::MagneticField magnetometer_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, magnetometer_msg);
  magnetometerCallback_(magnetometer_msg);
  return;
};

void IgnitionBridge::ignitionPoseStaticCallback(const ignition::msgs::Pose_V &msg) {
  tf2_msgs::msg::TFMessage pose_static_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, pose_static_msg);

  for (int i = 0; i < pose_static_msg.transforms.size(); i++) {
    geometry_msgs::msg::TransformStamped transform_stamped = pose_static_msg.transforms[i];
    if (("/" + transform_stamped.header.frame_id) == name_space_ &&
        ("/" + transform_stamped.child_frame_id) != (name_space_ + "/base_link")) {
      std::string parent_id = transform_stamped.header.frame_id;
      std::string child_id = transform_stamped.child_frame_id;
      std::string sensor_name = child_id.substr(parent_id.length() + 1);

      transform_stamped.header.frame_id = name_space_ + "/base_link";
      transform_stamped.child_frame_id = name_space_ + "/" + sensor_name;

      auto callback = callbacks_sensors_transform_.find(sensor_name);
      if (callback != callbacks_sensors_transform_.end()) {
        callback->second(transform_stamped, sensor_name);
      }
    }
  }
  return;
};

void IgnitionBridge::unsuscribePoseStatic() {
  std::string topic = "model" + name_space_ + "/pose_static";
  ign_node_ptr_->Unsubscribe(topic);
  return;
};

// Cameras
void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type, cameraCallbackType cameraCallback,
                               cameraInfoCallbackType cameraInfoCallback,
                               tfCallbackType poseStaticCallback) {
  std::string camera_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                             sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                             "/image";
  callbacks_camera_.insert(std::make_pair(camera_topic, cameraCallback));
  callbacks_sensors_names_.insert(std::make_pair(camera_topic, sensor_name));
  ign_node_ptr_->Subscribe(camera_topic, IgnitionBridge::ignitionCameraCallback);

  std::string camera_info_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                                  sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                                  "/camera_info";
  callbacks_camera_info_.insert(std::make_pair(camera_info_topic, cameraInfoCallback));
  callbacks_sensors_names_.insert(std::make_pair(camera_info_topic, sensor_name));
  ign_node_ptr_->Subscribe(camera_info_topic, IgnitionBridge::ignitionCameraInfoCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

void IgnitionBridge::ignitionCameraCallback(const ignition::msgs::Image &msg,
                                            const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::Image ros_image_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, ros_image_msg);
  auto callback = callbacks_camera_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());

  static auto frame_id =
      refactorizeFrameId(ros_image_msg.header.frame_id, sensor_name->second, "camera_link");
  ros_image_msg.header.frame_id = frame_id;

  if (callback != callbacks_camera_.end()) {
    callback->second(ros_image_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::ignitionCameraInfoCallback(const ignition::msgs::CameraInfo &msg,
                                                const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::CameraInfo ros_camera_info_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, ros_camera_info_msg);
  auto callback = callbacks_camera_info_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());

  static auto frame_id =
      refactorizeFrameId(ros_camera_info_msg.header.frame_id, sensor_name->second, "camera_link");
  ros_camera_info_msg.header.frame_id = frame_id;

  // std::cout << "final_frame_id: " << final_frame_id << std::endl;

  if (callback != callbacks_camera_info_.end()) {
    callback->second(ros_camera_info_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type, laserScanCallbackType laserScanCallback,
                               pointCloudCallbackType pointCloudCallback,
                               tfCallbackType poseStaticCallback) {
  std::string laser_scan_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                                 sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                                 "/scan";
  callbacks_laser_scan_.insert(std::make_pair(laser_scan_topic, laserScanCallback));
  callbacks_sensors_names_.insert(std::make_pair(laser_scan_topic, sensor_name));
  ign_node_ptr_->Subscribe(laser_scan_topic, IgnitionBridge::ignitionLaserScanCallback);

  std::string point_cloud_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                                  sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                                  "/scan/points";
  callbacks_point_cloud_.insert(std::make_pair(point_cloud_topic, pointCloudCallback));
  callbacks_sensors_names_.insert(std::make_pair(point_cloud_topic, sensor_name));
  ign_node_ptr_->Subscribe(point_cloud_topic, IgnitionBridge::ignitionPointCloudCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

void IgnitionBridge::ignitionLaserScanCallback(const ignition::msgs::LaserScan &msg,
                                               const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::LaserScan ros_laser_scan_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, ros_laser_scan_msg);
  auto callback = callbacks_laser_scan_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_laser_scan_.end()) {
    callback->second(ros_laser_scan_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::ignitionPointCloudCallback(const ignition::msgs::PointCloudPacked &msg,
                                                const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::PointCloud2 ros_point_cloud_msg;
  ros_ign_bridge::convert_ign_to_ros(msg, ros_point_cloud_msg);
  auto callback = callbacks_point_cloud_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_point_cloud_.end()) {
    callback->second(ros_point_cloud_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type, gpsCallbackType gpsCallback,
                               tfCallbackType poseStaticCallback) {
  std::string gps_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                          sensor_name + "/link/" + link_name + "/sensor/" + sensor_type + "/navsat";
  callbacks_gps_.insert(std::make_pair(gps_topic, gpsCallback));
  callbacks_sensors_names_.insert(std::make_pair(gps_topic, sensor_name));
  ign_node_ptr_->Subscribe(gps_topic, IgnitionBridge::ignitionGPSCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

std::string replace_delimiter(const std::string &input, const std::string &old_delim,
                              const std::string new_delim) {
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size()) {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos) {
      output += new_delim;
      pos += old_delim.size();
    }
    last_pos = pos;
  }
  return output;
};

void IgnitionBridge::ignitionGPSCallback(const ignition::msgs::NavSat &ign_msg,
                                         const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::NavSatFix ros_msg;
  // ros_ign_bridge::convert_ign_to_ros(msg, ros_gps_msg);

  ros_ign_bridge::convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = replace_delimiter(ign_msg.frame_id(), "::", "/");
  ros_msg.latitude = ign_msg.latitude_deg();
  ros_msg.longitude = ign_msg.longitude_deg();
  ros_msg.altitude = ign_msg.altitude();

  // position_covariance is not supported in Ignition::Msgs::NavSat.
  ros_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  auto callback = callbacks_gps_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_gps_.end()) {
    callback->second(ros_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type, imuCallbackType imuCallback,
                               tfCallbackType poseStaticCallback) {
  std::string imu_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                          sensor_name + "/link/" + link_name + "/sensor/" + sensor_type + "/navsat";
  callbacks_imu_.insert(std::make_pair(imu_topic, imuCallback));
  callbacks_sensors_names_.insert(std::make_pair(imu_topic, sensor_name));
  ign_node_ptr_->Subscribe(imu_topic, IgnitionBridge::ignitionGPSCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

void IgnitionBridge::ignitionImuCallback(const ignition::msgs::IMU &ign_msg,
                                         const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::Imu ros_imu_msg;
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_imu_msg);
  auto callback = callbacks_imu_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_imu_.end()) {
    callback->second(ros_imu_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type,
                               airPressureCallbackType air_pressureCallback,
                               tfCallbackType poseStaticCallback) {
  std::string air_pressure_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                                   sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                                   "/navsat";
  callbacks_air_pressure_.insert(std::make_pair(air_pressure_topic, air_pressureCallback));
  callbacks_sensors_names_.insert(std::make_pair(air_pressure_topic, sensor_name));
  ign_node_ptr_->Subscribe(air_pressure_topic, IgnitionBridge::ignitionGPSCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

void IgnitionBridge::ignitionAirPressureCallback(const ignition::msgs::FluidPressure &ign_msg,
                                                 const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::FluidPressure ros_air_pressure_msg;
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_air_pressure_msg);
  auto callback = callbacks_air_pressure_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_air_pressure_.end()) {
    callback->second(ros_air_pressure_msg, sensor_name->second);
  }
  return;
};

void IgnitionBridge::addSensor(std::string world_name, std::string name_space,
                               std::string sensor_name, std::string link_name,
                               std::string sensor_type,
                               magnetometerCallbackType magnetometerCallback,
                               tfCallbackType poseStaticCallback) {
  std::string magnetometer_topic = "/world/" + world_name + "/model/" + name_space + "/model/" +
                                   sensor_name + "/link/" + link_name + "/sensor/" + sensor_type +
                                   "/navsat";
  callbacks_magnetometer_.insert(std::make_pair(magnetometer_topic, magnetometerCallback));
  callbacks_sensors_names_.insert(std::make_pair(magnetometer_topic, sensor_name));
  ign_node_ptr_->Subscribe(magnetometer_topic, IgnitionBridge::ignitionGPSCallback);

  callbacks_sensors_transform_.insert(std::make_pair(sensor_name, poseStaticCallback));
  return;
};

void IgnitionBridge::ignitionMagnometerCallback(const ignition::msgs::Magnetometer &ign_msg,
                                                const ignition::transport::MessageInfo &msg_info) {
  sensor_msgs::msg::MagneticField ros_magnetometer_msg;
  ros_ign_bridge::convert_ign_to_ros(ign_msg, ros_magnetometer_msg);
  auto callback = callbacks_magnetometer_.find(msg_info.Topic());
  auto sensor_name = callbacks_sensors_names_.find(msg_info.Topic());
  if (callback != callbacks_magnetometer_.end()) {
    callback->second(ros_magnetometer_msg, sensor_name->second);
  }
  return;
};
}  // namespace ignition_platform
