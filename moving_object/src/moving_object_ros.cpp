
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
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include "moving_object/moving_object_ros.hpp"

namespace moving_object
{
MovingObjectRos::MovingObjectRos(const std::shared_ptr<Param>& param) : Node("moving_object")
{
  RCLCPP_INFO(get_logger(), "Entering MovingObjectRos Constructor...");

  if (param == nullptr)
  {
    params_ = std::make_shared<Param>();
  }
  else
  {
    params_ = param;
  }

  f_detection_sub_ = std::make_unique<FilteredDetection>(this, params_->msg_object_detection_);
  f_tracking_sub_ = std::make_unique<FilteredTracking>(this, params_->msg_object_tracking_);
  f_localization_sub_ =
      std::make_unique<FilteredLocalization>(this, params_->msg_object_localization_);

  sync_sub_ =
      std::make_unique<FilteredSync>(*f_detection_sub_, *f_tracking_sub_, *f_localization_sub_, 10);

  sync_sub_->registerCallback(&MovingObjectRos::onObjectsReceived, this);

  RCLCPP_INFO(get_logger(), "...Creating Moving Objects buffer...");
  frames_ = std::make_shared<MovingObjects>(rclcpp::Node::SharedPtr(this), params_);
  RCLCPP_INFO(get_logger(), "...message_detction:%s, tracking:%s, localization:%s",
              params_->msg_object_detection_.c_str(), params_->msg_object_tracking_.c_str(),
              params_->msg_object_localization_.c_str());
}

MovingObjectRos::~MovingObjectRos()
{
}

void MovingObjectRos::onObjectsReceived(const DetectionMsg::SharedPtr& detect,
                                        const TrackingMsg::SharedPtr& track,
                                        const LocalizationMsg::SharedPtr& loc)
{
  if (loc->header.stamp != track->header.stamp || track->header.stamp != detect->header.stamp ||
      loc->header.stamp != detect->header.stamp)
  {
    RCLCPP_WARN(get_logger(), "...Doesn't meet the stamp check, do nothing for the current \
    messages");
    RCLCPP_WARN(get_logger(), "......D==%ld.%ld, T==%ld.%ld, L==%ld.%ld", detect->header.stamp.sec,
                detect->header.stamp.nanosec, track->header.stamp.sec, track->header.stamp.nanosec,
                loc->header.stamp.sec, loc->header.stamp.nanosec);

    return;
  }
  if (loc->header.frame_id != track->header.frame_id ||
      track->header.frame_id != detect->header.frame_id ||
      loc->header.frame_id != detect->header.frame_id)
  {
    RCLCPP_WARN(get_logger(), "...Doesn't meet the frame_id check, do nothing for the current \
messages");

    return;
  }

  mutex_.lock();
  frames_->processFrame(detect, track, loc);
  mutex_.unlock();
}

bool MovingObjectRos::setParameters(const std::shared_ptr<Param> params)
{
  if (params != nullptr)
  {
    params_ = params;
    return true;
  }
  return false;
}

}  // namespace

#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(moving_object::MovingObjectRos, rclcpp::Node)