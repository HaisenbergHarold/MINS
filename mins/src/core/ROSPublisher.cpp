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

#include "ROSPublisher.h"
#include "options/OptionsCamera.h"
#include "options/OptionsGPS.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/LidarTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std;
using namespace Eigen;

namespace mins {

ROSPublisher::ROSPublisher(shared_ptr<rclcpp::Node> node) : node_(node) {
  // ------------------------------------------------
  // General publishers
  // ------------------------------------------------
  pub_odom_imu = node_->create_publisher<nav_msgs::msg::Odometry>("/mins/odom_imu", rclcpp::QoS(10));
  pub_pose_imu = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/mins/pose_imu", rclcpp::QoS(10));
  pub_path_imu = node_->create_publisher<nav_msgs::msg::Path>("/mins/path_imu", rclcpp::QoS(10));
  pub_loop_pose = node_->create_publisher<nav_msgs::msg::Odometry>("/mins/loop_odom", rclcpp::QoS(10));
  pub_state_debug = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/mins/state_debug", rclcpp::QoS(10));

  // ------------------------------------------------
  // TF2 broadcaster
  // ------------------------------------------------
  tf_br_ = make_shared<tf2_ros::TransformBroadcaster>(node_);

  // ------------------------------------------------
  // GPS publishers (conditionally created later at publish time)
  // ------------------------------------------------
  pub_gps_mm = nullptr;
  pub_gps_fix = nullptr;
  pub_gps_cloud = nullptr;

  // ------------------------------------------------
  // LiDAR map publisher
  // ------------------------------------------------
  pub_lidar_map = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/mins/lidar_map", rclcpp::QoS(10));

  // camera publishers are created per camera
  pub_cam_images.clear();
}

// =======================================================================
// publish_odometry
// =======================================================================
void ROSPublisher::publish_odometry(const Matrix<double, 13, 1> &odom, const vector<shared_ptr<ov_type::Type>> &order,
                                    int active_tracks) {
  auto msg = ROSHelper::ToOdometry(odom);
  msg.header.stamp = node_->now();
  msg.header.frame_id = "global";
  msg.child_frame_id = "imu";
  pub_odom_imu->publish(msg);
}

// =======================================================================
// publish_pose
// =======================================================================
void ROSPublisher::publish_pose(const Matrix<double, 7, 1> &pose_in, const Matrix<double, 6, 6> &cov_in) {
  auto msg = ROSHelper::ToPoseWithCovarianceStamped(pose_in, cov_in);
  msg.header.stamp = node_->now();
  msg.header.frame_id = "global";
  pub_pose_imu->publish(msg);
}

// =======================================================================
// publish_imu_path
// =======================================================================
void ROSPublisher::publish_imu_path(const Matrix<double, 3, 1> &pos, const Matrix<double, 4, 1> &quat) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = "global";
  pose.pose.position.x = pos(0);
  pose.pose.position.y = pos(1);
  pose.pose.position.z = pos(2);
  pose.pose.orientation.w = quat(0);
  pose.pose.orientation.x = quat(1);
  pose.pose.orientation.y = quat(2);
  pose.pose.orientation.z = quat(3);
  poses_imu.push_back(pose);
  auto path_msg = ROSHelper::ToNavPath(poses_imu);
  path_msg.header.stamp = node_->now();
  path_msg.header.frame_id = "global";
  pub_path_imu->publish(path_msg);
}

// =======================================================================
// publish_lidar_map
// =======================================================================
void ROSPublisher::publish_lidar_map() {
  // Lidar map publishing requires UpdaterLidar data not in State.
  // TODO: Connect to UpdaterLidar for map visualization.
  return;
}

// =======================================================================
// publish_gps
// =======================================================================
void ROSPublisher::publish_gps(const GPSData &gps) {
  // Lazy-create GPS publishers
  if (pub_gps_mm == nullptr) {
    pub_gps_mm = node_->create_publisher<visualization_msgs::msg::Marker>("/mins/gps_marker", rclcpp::QoS(10));
    pub_gps_fix = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/mins/gps_fix", rclcpp::QoS(10));
    pub_gps_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/mins/gps_cloud", rclcpp::QoS(10));
  }

  // Publish GPS fix
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = node_->now();
  fix.header.frame_id = "global";
  fix.latitude = gps.meas(0);
  fix.longitude = gps.meas(1);
  fix.altitude = gps.meas(2);
  fix.position_covariance[0] = gps.noise(0);
  fix.position_covariance[4] = gps.noise(1);
  fix.position_covariance[8] = gps.noise(2);
  pub_gps_fix->publish(fix);

  // Publish GPS marker
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->now();
  marker.header.frame_id = "global";
  marker.ns = "gps";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = gps.meas(0);
  marker.pose.position.y = gps.meas(1);
  marker.pose.position.z = gps.meas(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  pub_gps_mm->publish(marker);
}

// =======================================================================
// publish_loop_closure_pose
// =======================================================================
void ROSPublisher::publish_loop_closure_pose(const Matrix<double, 7, 1> &pose, const Matrix<double, 6, 6> &cov) {
  auto msg = ROSHelper::ToPoseWithCovarianceStamped(pose, cov);
  msg.header.stamp = node_->now();
  msg.header.frame_id = "global";

  nav_msgs::msg::Odometry odom;
  odom.header = msg.header;
  odom.child_frame_id = "imu";
  odom.pose.pose = msg.pose.pose;
  for (int i = 0; i < 36; i++)
    odom.pose.covariance[i] = msg.pose.covariance[i];
  pub_loop_pose->publish(odom);
}

// =======================================================================
// publish_transform (ROS1 tf → ROS2 tf2)
// =======================================================================
void ROSPublisher::publish_transform(const shared_ptr<ov_type::PoseJPL> &pose) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = node_->now();
  transform_stamped.header.frame_id = "global";
  transform_stamped.child_frame_id = "imu";

  Vector4d quat = pose->quat();
  quat = quat(0) < 0 ? -quat : quat;
  transform_stamped.transform.translation.x = pose->pos()(0);
  transform_stamped.transform.translation.y = pose->pos()(1);
  transform_stamped.transform.translation.z = pose->pos()(2);
  transform_stamped.transform.rotation.w = quat(0);
  transform_stamped.transform.rotation.x = quat(1);
  transform_stamped.transform.rotation.y = quat(2);
  transform_stamped.transform.rotation.z = quat(3);

  tf_br_->sendTransform(transform_stamped);
}

// =======================================================================
// save_trajectory_to_file
// =======================================================================
void ROSPublisher::save_trajectory_to_file() {
  // Save to file
  string file_path = "/tmp/mins_trajectory.txt";
  ofstream file(file_path);
  if (!file.is_open()) {
    PRINT1(RED "[PUB] Unable to open trajectory file: %s\n" RESET, file_path.c_str());
    return;
  }

  file << "# timestamp tx ty tz qx qy qz qw" << endl;
  for (auto &pose : poses_imu) {
    file << fixed << pose.header.stamp.sec << "." << pose.header.stamp.nanosec << " ";
    file << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z << " ";
    file << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " "
         << pose.pose.orientation.w << endl;
  }
  file.close();
  PRINT1(GREEN "[PUB] Trajectory saved to %s\n" RESET, file_path.c_str());
}

} // namespace mins