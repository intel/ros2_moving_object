
/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <ament_index_cpp/get_resource.hpp>
#include "moving_object/moving_object_ros.hpp"
#include "moving_object/param.hpp"

void print_usage()
{
  printf("Usage for moving_object app:\n");
  printf("moving_object [-h] [-c config-path-file]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-c : Use cofig-path-file (in .yaml format) as the config file.\n");
}

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
  {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);

  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "moving_object", content, &prefix_path);

  // Parse the command line options.
  auto config = std::string(prefix_path + "/share/moving_object/param/moving_object.yaml");
  if (rcutils_cli_option_exist(argv, argv + argc, "-c"))
  {
    config = std::string(rcutils_cli_get_option(argv, argv + argc, "-c"));
  }

  auto params = std::make_shared<moving_object::Param>(config);
  auto node = std::make_shared<moving_object::MovingObjectRos>(params);
  RCLCPP_INFO(node->get_logger(), "ENTER moving_object ROS node. parameter config=[%s] ",
              config.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return (0);
}
