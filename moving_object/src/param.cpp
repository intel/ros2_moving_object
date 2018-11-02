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

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "moving_object/param.hpp"

namespace moving_object
{
#ifndef PARAM_ERROR
#define PARAM_ERROR(str) std::cout << "[PARAM_ERROR]: " << str << std::endl
#endif
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  try
  {
    i = node.as<T>();
  }
  catch (YAML::InvalidScalar)
  {
    PARAM_ERROR("The YAML file does not contain a device_index tag [" << node.Tag() << "] or it is "
                                                                                     "invalid.");
  }
}
Param::Param()
{
  init();
}

Param::Param(const std::string& file_path)
{
  init();
  loadParamFromYAML(file_path);
}

void Param::init()
{
  social_filtering_enabled_ = kObjectFiltering_TypeFiltering_Enabling;
  moving_object_msg_enabled_ = true; // should be true.
  posibility_threshold_ = kObjectFiltering_PosibilityThreshold;
  overlap_ratio_threshold_ = kObject_OverlapRatioThreshold;
  max_frames_ = kFrameCache_Size;
  velocity_enabled_ = kVelocityCalculation_Enabling;
  fixed_frame_ = kVelocityCalculation_FixedFrame;
  msg_object_detection_ = kMessage_ObjectDetection;
  msg_object_tracking_ = kMessage_ObjectTracking;
  msg_object_localization_ = kMessage_ObjectLocalization;
}

bool Param::loadParamFromYAML(const std::string& file_path)
{
  std::ifstream fin(file_path);
  if (fin.fail())
  {
    PARAM_ERROR("Could not open config file:" << file_path);
    return false;
  }

  YAML::Node doc = YAML::Load(fin);

  doc["social_filtering_enabled"] >> social_filtering_enabled_;
  doc["moving_object_msg_enabled"] >> moving_object_msg_enabled_;
  doc["posibility_threshold"] >> posibility_threshold_;
  doc["overlap_ratio_threshold"] >> overlap_ratio_threshold_;
  doc["max_frames"] >> max_frames_;
  doc["velocity_enabled"] >> velocity_enabled_;
  doc["fixed_frame"] >> fixed_frame_;
  doc["msg_object_detection"] >> msg_object_detection_;
  doc["msg_object_tracking"] >> msg_object_tracking_;
  doc["msg_object_localization"] >> msg_object_localization_;

  return true;
}

bool Param::validateParam()
{
  /**< TODO todo: Add validation criteria here. */

  return true;
}
}