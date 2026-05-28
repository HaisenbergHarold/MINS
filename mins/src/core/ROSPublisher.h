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

#ifndef MINS_ROSPUBLISHER_H
#define MINS_ROSPUBLISHER_H

#include "ROSHelper.h"
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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <string>

using namespace Eigen;
using namespace std;
using namespace ov_type;

namespace mins {

class ROSPublisher {

public:
  /// Creator: loads settings
  ROSPublisher(shared_ptr<rclcpp::Node> node);

  /// Publish functions
  void publish_odometry(const Matrix<double, 13, 1> &odom, const vector<shared_ptr<ov_type::Type>> &order, int active_tracks = -1);
  void publish_pose(const Matrix<double, 7, 1> &pose_in, const Matrix<double, 6, 6> &cov_in);
  void publish_imu_path(const Matrix<double, 3, 1> &pos, const Matrix<double, 4, 1> &quat);
  void publish_lidar_map();
  void publish_gps(const GPSData &gps);
  void publish_loop_closure_pose(const Eigen::Matrix<double, 7, 1> &pose, const Eigen::Matrix<double, 6, 6> &cov);

  /// TF2: publish transform
  void publish_transform(const shared_ptr<ov_type::PoseJPL> &pose);

  /// Save path and trajectory
  void save_trajectory_to_file();

  /// State and its helpers
  shared_ptr<State> state;
  shared_ptr<OptionsCamera> op_camera;
  shared_ptr<OptionsGPS> op_gps;

  /// Counters
  int frames = 0;

protected:
  /// Node
  shared_ptr<rclcpp::Node> node_;

  /// Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_imu;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_imu;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_gt;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_imu;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_map;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_loop_pose;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_gps_mm;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_fix;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gps_cloud;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_debug;

  /// Camera publishers
  vector<shared_ptr<image_transport::Publisher>> pub_cam_images;

  /// TF2 broadcaster
  shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  /// Path
  vector<geometry_msgs::msg::PoseStamped> poses_imu;
};

} // namespace mins

#endif // MINS_ROSPUBLISHER_H