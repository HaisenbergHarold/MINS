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

#include <memory>

#include "core/ROSPublisher.h"
#include "core/ROSSubscriber.h"
#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsSystem.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace mins;

// Main function
int main(int argc, char **argv) {

  // Launch our ros2 node
  rclcpp::init(argc, argv);
  auto node = make_shared<rclcpp::Node>("mins_subscribe");

  // Ensure we have a path, if the user passes it then we should use it
  string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }
  node->declare_parameter<string>("config_path", config_path);
  node->get_parameter("config_path", config_path);

  // Load the config
  auto parser = make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node);
  shared_ptr<Options> op = make_shared<Options>();
  op->load_print(parser);
  shared_ptr<State_Logger> save = make_shared<State_Logger>(op);

  // Create our system
  shared_ptr<SystemManager> sys = make_shared<SystemManager>(op->est);
  shared_ptr<ROSPublisher> pub = make_shared<ROSPublisher>(node);
  pub->state = sys->state;
  pub->op_camera = op->est->cam;
  pub->op_gps = op->est->gps;
  shared_ptr<ROSSubscriber> sub = make_shared<ROSSubscriber>(node, sys, op->est);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  // Spin off to ROS2
  rclcpp::spin(node);

  // Final visualization
  sys->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();

  rclcpp::shutdown();

  // Done!
  return EXIT_SUCCESS;
}
