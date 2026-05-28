/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "ROSHelper.h"
#include "options/OptionsCamera.h"
#include "types/PoseJPL.h"
#include "update/gps/GPSTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using namespace Eigen;
using namespace mins;

// =======================================================================
// IMU: sensor_msgs::msg::Imu → ov_core::ImuData (unchanged from ROS1)
// =======================================================================
ov_core::ImuData ROSHelper::Imu2Data(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
  ov_core::ImuData imu;

  // Convert to double
  imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();

  // Copy the linear acceleration
  imu.am(0) = msg->linear_acceleration.x;
  imu.am(1) = msg->linear_acceleration.y;
  imu.am(2) = msg->linear_acceleration.z;

  // Copy the angular velocity
  imu.wm(0) = msg->angular_velocity.x;
  imu.wm(1) = msg->angular_velocity.y;
  imu.wm(2) = msg->angular_velocity.z;

  return imu;
}

// =======================================================================
// DaoYuan IMU → ov_core::ImuData
// DaoYuanImu.msg fields:
//   float64[9] orientation    -- [qw, qx, qy, qz, pitch, roll, yaw, acc_x, acc_y]
//   float64[3] angular_velocity
//   float64[3] linear_acceleration
//   float64 altitude
//   int32 temperature
// =======================================================================
ov_core::ImuData ROSHelper::DaoYuanImu2Data(const gnss_msgs::msg::DaoYuanImu::ConstSharedPtr &msg) {
  ov_core::ImuData imu;

  imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();

  // Angular velocity [deg/s in DaoYuan] → convert to [rad/s]
  imu.wm(0) = msg->gyro_x * M_PI / 180.0;
  imu.wm(1) = msg->gyro_y * M_PI / 180.0;
  imu.wm(2) = msg->gyro_z * M_PI / 180.0;

  // Linear acceleration [m/s^2]
  imu.am(0) = msg->acc_x;
  imu.am(1) = msg->acc_y;
  imu.am(2) = msg->acc_z;

  return imu;
}

// =======================================================================
// DaoYuan GNSS → GPSData
// DaoYuanGnss.msg fields:
//   float64 latitude           -- [deg]
//   float64 longitude          -- [deg]
//   float64 altitude           -- [m]
//   float64[9] orientation     -- [qw, qx, qy, qz, pitch, roll, yaw]
//   float64[3] linear_velocity -- [vx, vy, vz] in m/s
//   float64[3] angular_velocity-- [wx, wy, wz] in deg/s
//   float64[3] position_covariance
//   float64[3] orientation_covariance
//   float64[3] linear_velocity_covariance
//   float64[3] angular_velocity_covariance
// =======================================================================
GPSData ROSHelper::DaoYuanGnss2Data(const gnss_msgs::msg::DaoYuanGnss::ConstSharedPtr &msg, int gps_id) {
  GPSData data;
  data.time = rclcpp::Time(msg->header.stamp).seconds();
  data.id = gps_id;

  // Latitude, longitude, height
  data.meas(0) = msg->latitude;
  data.meas(1) = msg->longitude;
  data.meas(2) = msg->height;

  // Noise (std values)
  data.noise(0) = msg->std_lat;
  data.noise(1) = msg->std_lon;
  data.noise(2) = msg->std_height;

  return data;
}

// =======================================================================
// NavSatFix → GPSData (standard GPS)
// =======================================================================
GPSData ROSHelper::NavSatFix2Data(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg, int gps_id) {
  GPSData data;
  data.time = rclcpp::Time(msg->header.stamp).seconds();
  data.id = gps_id;

  // Latitude, longitude, altitude
  data.meas(0) = msg->latitude;
  data.meas(1) = msg->longitude;
  data.meas(2) = msg->altitude;

  // Noise
  data.noise(0) = sqrt(msg->position_covariance[0]);
  data.noise(1) = sqrt(msg->position_covariance[4]);
  data.noise(2) = sqrt(msg->position_covariance[8]);

  return data;
}

// =======================================================================
// Camera: sensor_msgs::msg::Image → ov_core::CameraData
// =======================================================================
bool ROSHelper::Image2Data(const sensor_msgs::msg::Image::ConstSharedPtr &msg, int cam_id, ov_core::CameraData &cam,
                           std::shared_ptr<OptionsCamera> op) {

  // Convert the image to OpenCV
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT1(RED "[ROSHELPER] cv_bridge exception: %s\n" RESET, e.what());
    return false;
  }

  // Create our camera data
  cam.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  cam.images.push_back(cv_ptr->image.clone());
  cam.sensor_ids.push_back(cam_id);

  return true;
}

// =======================================================================
// Wheel: sensor_msgs::msg::JointState → WheelData
// =======================================================================
WheelData ROSHelper::JointState2Data(const sensor_msgs::msg::JointState::ConstSharedPtr &msg) {
  WheelData data;
  data.time = rclcpp::Time(msg->header.stamp).seconds();

  if (msg->velocity.size() >= 1)
    data.m1 = msg->velocity.at(0);
  if (msg->velocity.size() >= 2)
    data.m2 = msg->velocity.at(1);

  return data;
}

// =======================================================================
// LiDAR: sensor_msgs::msg::PointCloud2 → pcl::PointCloud<pcl::PointXYZ>
// =======================================================================
shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ROSHelper::rosPC2pclPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar_id) {
  auto cloud = make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);
  return cloud;
}

// =======================================================================
// State publishing helpers
// =======================================================================

rclcpp::Time ROSHelper::ros_time_from_seconds(double seconds) {
  int64_t sec = static_cast<int64_t>(seconds);
  uint64_t nsec = static_cast<uint64_t>((seconds - sec) * 1e9);
  return rclcpp::Time(sec, nsec);
}

builtin_interfaces::msg::Time ROSHelper::ros_time_msg_from_seconds(double seconds) {
  builtin_interfaces::msg::Time t;
  int64_t sec = static_cast<int64_t>(seconds);
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
  return t;
}

geometry_msgs::msg::PoseStamped ROSHelper::ToPoseStamped(const Matrix<double, 7, 1> &pose_in) {
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = pose_in(0);
  pose.pose.position.y = pose_in(1);
  pose.pose.position.z = pose_in(2);
  pose.pose.orientation.w = pose_in(3);
  pose.pose.orientation.x = pose_in(4);
  pose.pose.orientation.y = pose_in(5);
  pose.pose.orientation.z = pose_in(6);
  return pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped
ROSHelper::ToPoseWithCovarianceStamped(const Matrix<double, 7, 1> &pose_in, const Matrix<double, 6, 6> &cov_in) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = pose_in(0);
  pose.pose.pose.position.y = pose_in(1);
  pose.pose.pose.position.z = pose_in(2);
  pose.pose.pose.orientation.w = pose_in(3);
  pose.pose.pose.orientation.x = pose_in(4);
  pose.pose.pose.orientation.y = pose_in(5);
  pose.pose.pose.orientation.z = pose_in(6);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      pose.pose.covariance[6 * r + c] = cov_in(r, c);
    }
  }
  return pose;
}

nav_msgs::msg::Odometry ROSHelper::ToOdometry(const Matrix<double, 13, 1> &odom_in) {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = odom_in(4);
  odom.pose.pose.position.y = odom_in(5);
  odom.pose.pose.position.z = odom_in(6);
  odom.pose.pose.orientation.w = odom_in(0);
  odom.pose.pose.orientation.x = odom_in(1);
  odom.pose.pose.orientation.y = odom_in(2);
  odom.pose.pose.orientation.z = odom_in(3);
  odom.twist.twist.linear.x = odom_in(7);
  odom.twist.twist.linear.y = odom_in(8);
  odom.twist.twist.linear.z = odom_in(9);
  odom.twist.twist.angular.x = odom_in(10);
  odom.twist.twist.angular.y = odom_in(11);
  odom.twist.twist.angular.z = odom_in(12);
  return odom;
}

nav_msgs::msg::Path ROSHelper::ToNavPath(const vector<geometry_msgs::msg::PoseStamped> &poses) {
  nav_msgs::msg::Path path;
  path.poses = poses;
  return path;
}

// =======================================================================
// TF helpers (ROS1 tf → ROS2 tf2)
// =======================================================================
tf2::Stamped<tf2::Transform> ROSHelper::Pose2TF(const shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {
  tf2::Transform transform;
  if (flip_trans) {
    Matrix<double, 3, 1> pos = -pose->Rot() * pose->pos();
    Vector4d quat = pose->quat();
    quat = quat(0) < 0 ? -quat : quat;
    transform.setOrigin(tf2::Vector3(pos(0), pos(1), pos(2)));
    transform.setRotation(tf2::Quaternion(quat(1), quat(2), quat(3), quat(0)));
  } else {
    Vector4d quat = pose->quat();
    quat = quat(0) < 0 ? -quat : quat;
    transform.setOrigin(tf2::Vector3(pose->pos()(0), pose->pos()(1), pose->pos()(2)));
    transform.setRotation(tf2::Quaternion(quat(1), quat(2), quat(3), quat(0)));
  }
  tf2::Stamped<tf2::Transform> stamped(transform, tf2::TimePointZero, "global");
  return stamped;
}

tf2::Stamped<tf2::Transform> ROSHelper::Pos2TF(const Eigen::Vector3d &pos, bool flip_trans) {
  tf2::Transform transform;
  if (flip_trans) {
    transform.setOrigin(tf2::Vector3(-pos(0), -pos(1), -pos(2)));
  } else {
    transform.setOrigin(tf2::Vector3(pos(0), pos(1), pos(2)));
  }
  transform.setRotation(tf2::Quaternion::getIdentity());
  tf2::Stamped<tf2::Transform> stamped(transform, tf2::TimePointZero, "global");
  return stamped;
}