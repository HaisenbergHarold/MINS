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

#ifndef MINS_ROSSUBSCRIBER_H
#define MINS_ROSSUBSCRIBER_H

#include "ROSHelper.h"
#include "core/SystemManager.h"
#include "options/OptionsCamera.h"
#include "options/OptionsGPS.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "utils/Print_Logger.h"
#include <Eigen/Eigen>
#include <gnss_msgs/msg/dao_yuan_gnss.hpp>
#include <gnss_msgs/msg/dao_yuan_imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <string>

using namespace std;
using namespace Eigen;

namespace mins {

class ROSSubscriber {

public:
    /// Creator: loads settings
  ROSSubscriber(shared_ptr<rclcpp::Node> node, shared_ptr<SystemManager> sys, shared_ptr<OptionsEstimator> op);

  /// Callbacks
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void callback_wheel(const sensor_msgs::msg::JointState::ConstSharedPtr &msg);
  void callback_gps(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg);
  void callback_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void callback_camera(const sensor_msgs::msg::Image::ConstSharedPtr &msg0, const sensor_msgs::msg::Image::ConstSharedPtr &msg1);

  /// DaoYuan callbacks: GNSS and IMU
  void callback_daoyuan_gnss(const gnss_msgs::msg::DaoYuanGnss::ConstSharedPtr &msg);
  void callback_daoyuan_imu(const gnss_msgs::msg::DaoYuanImu::ConstSharedPtr &msg);

  /// System manager and options
  shared_ptr<SystemManager> sys;
  shared_ptr<OptionsEstimator> options;

protected:
  /// Node
  shared_ptr<rclcpp::Node> node_;

  /// Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheel;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;

  /// DaoYuan subscribers
  rclcpp::Subscription<gnss_msgs::msg::DaoYuanGnss>::SharedPtr sub_daoyuan_gnss;
  rclcpp::Subscription<gnss_msgs::msg::DaoYuanImu>::SharedPtr sub_daoyuan_imu;

  /// Camera subscribers (image_transport + synchronization)
  shared_ptr<image_transport::ImageTransport> it_;
  image_transport::SubscriberFilter sub_cam0, sub_cam1;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol_cam;
  shared_ptr<message_filters::Synchronizer<sync_pol_cam>> sync_cam;
};

} // namespace mins

#endif // MINS_ROSSUBSCRIBER_H