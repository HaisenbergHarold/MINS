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

#include "ROSSubscriber.h"
#include "core/SystemManager.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/LidarTypes.h"
#include "utils/Print_Logger.h"
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace mins;

ROSSubscriber::ROSSubscriber(shared_ptr<rclcpp::Node> node, shared_ptr<SystemManager> sys, shared_ptr<OptionsEstimator> op)
    : node_(node), sys(sys), options(op) {

  // =======================================================================
  // IMU subscriber
  // =======================================================================
  if (!options->node_imu_topic.empty()) {
    sub_imu = node_->create_subscription<sensor_msgs::msg::Imu>(options->node_imu_topic, rclcpp::QoS(100),
                                                                std::bind(&ROSSubscriber::callback_imu, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to IMU: %s\n" RESET, options->node_imu_topic.c_str());
  }

  // =======================================================================
  // Wheel subscriber
  // =======================================================================
  if (!options->node_wheel_topic.empty()) {
    sub_wheel = node_->create_subscription<sensor_msgs::msg::JointState>(
        options->node_wheel_topic, rclcpp::QoS(10), std::bind(&ROSSubscriber::callback_wheel, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to Wheel: %s\n" RESET, options->node_wheel_topic.c_str());
  }

  // =======================================================================
  // GPS subscriber (standard NavSatFix)
  // =======================================================================
  if (!options->node_gps_topic.empty()) {
    sub_gps = node_->create_subscription<sensor_msgs::msg::NavSatFix>(options->node_gps_topic, rclcpp::QoS(10),
                                                                      std::bind(&ROSSubscriber::callback_gps, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to GPS (NavSatFix): %s\n" RESET, options->node_gps_topic.c_str());
  }

  // =======================================================================
  // DaoYuan GPS subscriber
  // =======================================================================
  if (!options->node_daoyuan_gps_topic.empty()) {
    sub_daoyuan_gnss = node_->create_subscription<gnss_msgs::msg::DaoYuanGnss>(
        options->node_daoyuan_gps_topic, rclcpp::QoS(10),
        std::bind(&ROSSubscriber::callback_daoyuan_gnss, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to DaoYuan GPS: %s\n" RESET, options->node_daoyuan_gps_topic.c_str());
  }

  // =======================================================================
  // DaoYuan IMU subscriber
  // =======================================================================
  if (!options->node_daoyuan_imu_topic.empty()) {
    sub_daoyuan_imu = node_->create_subscription<gnss_msgs::msg::DaoYuanImu>(
        options->node_daoyuan_imu_topic, rclcpp::QoS(100),
        std::bind(&ROSSubscriber::callback_daoyuan_imu, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to DaoYuan IMU: %s\n" RESET, options->node_daoyuan_imu_topic.c_str());
  }

  // =======================================================================
  // LiDAR subscriber
  // =======================================================================
  if (!options->node_lidar_topic.empty()) {
    sub_lidar = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        options->node_lidar_topic, rclcpp::QoS(100),
        std::bind(&ROSSubscriber::callback_lidar, this, std::placeholders::_1));
    PRINT1(GREEN "[SUB] Subscribing to LiDAR: %s\n" RESET, options->node_lidar_topic.c_str());
  }

  // =======================================================================
  // Camera subscribers (with synchronization via message_filters)
  // =======================================================================
  if (!options->node_camera_topic0.empty() && !options->node_camera_topic1.empty()) {
    sub_cam0.subscribe(node_.get(), options->node_camera_topic0, "raw");
    sub_cam1.subscribe(node_.get(), options->node_camera_topic1, "raw");
    sync_cam = make_shared<message_filters::Synchronizer<sync_pol_cam>>(sync_pol_cam(10), sub_cam0, sub_cam1);
    sync_cam->registerCallback(&ROSSubscriber::callback_camera, this);
    PRINT1(GREEN "[SUB] Subscribing to Cameras: %s & %s\n" RESET, options->node_camera_topic0.c_str(),
           options->node_camera_topic1.c_str());
  }
}

// =======================================================================
// IMU callback
// =======================================================================
void ROSSubscriber::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
  // Convert to our IMU data
  ov_core::ImuData imu = ROSHelper::Imu2Data(msg);
  sys->feed_measurement_imu(imu);
}

// =======================================================================
// Wheel callback
// =======================================================================
void ROSSubscriber::callback_wheel(const sensor_msgs::msg::JointState::ConstSharedPtr &msg) {
  WheelData wheel = ROSHelper::JointState2Data(msg);
  sys->feed_measurement_wheel(wheel);
}

// =======================================================================
// GPS callback (standard NavSatFix)
// =======================================================================
void ROSSubscriber::callback_gps(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
  for (int i = 0; i < options->gps->max_n; i++) {
    GPSData gps = ROSHelper::NavSatFix2Data(msg, i);
    if (options->gps->extrinsics.find(i) == options->gps->extrinsics.end())
      continue;

    // Apply GPS rotation extrinsic if configured
    if (options->gps->has_rotation) {
      // R33_imu_gnss converts from GNSS frame to IMU frame
      Matrix3d R_imu_gps = options->gps->R_imu_gps;
      Vector3d gps_imu = R_imu_gps * gps.meas;
      gps.meas = gps_imu;
    } else {
      // Apply translation only
      gps.meas -= options->gps->p_imu_gps;
    }

    sys->feed_measurement_gps(gps, true);
  }
}

// =======================================================================
// LiDAR callback
// =======================================================================
void ROSSubscriber::callback_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  for (int i = 0; i < options->lidar->max_n; i++) {
    auto cloud = ROSHelper::rosPC2pclPC(msg, i);
    sys->feed_measurement_lidar(cloud);
  }
}

// =======================================================================
// Camera callback (synchronized stereo)
// =======================================================================
void ROSSubscriber::callback_camera(const sensor_msgs::msg::Image::ConstSharedPtr &msg0,
                                    const sensor_msgs::msg::Image::ConstSharedPtr &msg1) {
  ov_core::CameraData cam0, cam1;
  bool success0 = ROSHelper::Image2Data(msg0, 0, cam0, options->cam);
  bool success1 = ROSHelper::Image2Data(msg1, 1, cam1, options->cam);
  if (success0 && success1) {
    // Merge
    cam0.images.push_back(cam1.images.at(0));
    cam0.sensor_ids.push_back(cam1.sensor_ids.at(0));
    sys->feed_measurement_camera(cam0);
  }
}

// =======================================================================
// DaoYuan GNSS callback
// - Converts DaoYuanGnss message → GPSData
// - Applies IMU-GNSS extrinsic (translation + rotation)
//
// DP010 example:
//   t31_imu_gnss = [0.453, 0.677, 1.790]
//   R33_imu_gnss = RPY[0, 0, 90] deg
//
// GPS measurement in GNSS frame: meas_gnss = [lat, lon, alt]
// Rotate to IMU frame: meas_imu = R_imu_gps * meas_gnss + p_imu_gps
// =======================================================================
void ROSSubscriber::callback_daoyuan_gnss(const gnss_msgs::msg::DaoYuanGnss::ConstSharedPtr &msg) {
  for (int i = 0; i < options->gps->max_n; i++) {
    if (options->gps->extrinsics.find(i) == options->gps->extrinsics.end())
      continue;

    GPSData gps = ROSHelper::DaoYuanGnss2Data(msg, i);

    // Apply IMU-GNSS extrinsic transformation
    if (options->gps->has_rotation) {
      // meas_gnss is [lat, lon, alt] in GNSS frame
      // Transform: meas_imu = R_imu_gps * (meas_gnss - p_imu_gps)
      Vector3d meas_imu = options->gps->R_imu_gps * gps.meas;
      if (options->gps->p_imu_gps.norm() > 1e-6) {
        meas_imu += options->gps->p_imu_gps;
      }
      gps.meas = meas_imu;
    } else {
      // Translation only
      gps.meas -= options->gps->p_imu_gps;
    }

    sys->feed_measurement_gps(gps, false);
  }
}

// =======================================================================
// DaoYuan IMU callback
// Converts DaoYuanImu message → ov_core::ImuData
// DaoYuan IMU typically provides angular velocity in deg/s and linear
// acceleration in m/s^2
// =======================================================================
void ROSSubscriber::callback_daoyuan_imu(const gnss_msgs::msg::DaoYuanImu::ConstSharedPtr &msg) {
  ov_core::ImuData imu = ROSHelper::DaoYuanImu2Data(msg);
  sys->feed_measurement_imu(imu);
}
