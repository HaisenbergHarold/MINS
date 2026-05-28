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

#ifndef MINS_ROSHELPER_H
#define MINS_ROSHELPER_H

#include "options/OptionsCamera.h"
#include "options/OptionsGPS.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/LidarTypes.h"
#include "update/wheel/WheelTypes.h"
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <image_transport/image_transport.hpp>
#include <gnss_msgs/msg/dao_yuan_gnss.hpp>
#include <gnss_msgs/msg/dao_yuan_imu.hpp>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

namespace mins {

/**
 * @brief Helper class that performs transformations between our types and ROS types.
 */
class ROSHelper {

public:
  // -----------------------------------------------------------------------
  // IMU (ROS1 sensor_msgs::Imu → ROS2 sensor_msgs::msg::Imu, no change)
  // -----------------------------------------------------------------------
  static ov_core::ImuData Imu2Data(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

  // -----------------------------------------------------------------------
  // DaoYuan GNSS → GPSData
  // -----------------------------------------------------------------------
  static GPSData DaoYuanGnss2Data(const gnss_msgs::msg::DaoYuanGnss::ConstSharedPtr &msg, int gps_id);

  // -----------------------------------------------------------------------
  // DaoYuan IMU → ov_core::ImuData
  // -----------------------------------------------------------------------
  static ov_core::ImuData DaoYuanImu2Data(const gnss_msgs::msg::DaoYuanImu::ConstSharedPtr &msg);

  // -----------------------------------------------------------------------
  // NavSatFix → GPSData (standard GPS)
  // -----------------------------------------------------------------------
  static GPSData NavSatFix2Data(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg, int gps_id);

  // -----------------------------------------------------------------------
  // Camera
  // -----------------------------------------------------------------------
  static bool Image2Data(const sensor_msgs::msg::Image::ConstSharedPtr &msg, int cam_id, ov_core::CameraData &cam, std::shared_ptr<OptionsCamera> op);

  // -----------------------------------------------------------------------
  // Wheel
  // -----------------------------------------------------------------------
  static WheelData JointState2Data(const sensor_msgs::msg::JointState::ConstSharedPtr &msg);

  // -----------------------------------------------------------------------
  // LiDAR
  // -----------------------------------------------------------------------
  static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rosPC2pclPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, int lidar_id);

  // -----------------------------------------------------------------------
  // State publishing
  // -----------------------------------------------------------------------
  static geometry_msgs::msg::PoseStamped ToPoseStamped(const Eigen::Matrix<double, 7, 1> &pose_in);
  static geometry_msgs::msg::PoseWithCovarianceStamped ToPoseWithCovarianceStamped(const Eigen::Matrix<double, 7, 1> &pose_in,
                                                                                   const Eigen::Matrix<double, 6, 6> &cov_in);
  static nav_msgs::msg::Odometry ToOdometry(const Eigen::Matrix<double, 13, 1> &odom_in);
  static nav_msgs::msg::Path ToNavPath(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

  // -----------------------------------------------------------------------
  // TF (ROS1 tf → ROS2 tf2)
  // -----------------------------------------------------------------------
  static tf2::Stamped<tf2::Transform> Pose2TF(const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);
  static tf2::Stamped<tf2::Transform> Pos2TF(const Eigen::Vector3d &pos, bool flip_trans);

private:
  static rclcpp::Time ros_time_from_seconds(double seconds);
  static builtin_interfaces::msg::Time ros_time_msg_from_seconds(double seconds);
};

} // namespace mins

#endif // MINS_ROSHELPER_H