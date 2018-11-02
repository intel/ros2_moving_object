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
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <moving_object_msgs/msg/moving_object.hpp>
#include <moving_object_msgs/msg/moving_objects_in_frame.hpp>
#include "moving_object/object_frame.hpp"
#include "moving_object/remap.hpp"

namespace moving_object
{
MovingObjectFrame::MovingObjectFrame(const builtin_interfaces::msg::Time& stamp,
                                     const std::string& frame_id,
                                     const rclcpp::Node::SharedPtr node,
                                     std::shared_ptr<Param> params)
  : node_(node), published_(false), params_(params)
{
  moving_objects_pub_ =
      node_->create_publisher<MovingObjectMsg>("/moving_object/moving_objects", 10);

  /**< @todo: configuring it by params for social_filter_ */
  social_filter_.clear();
  social_filter_ = kObjectFiltering_TypeFiltering_Types;

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  moving_objects_.clear();
  stamp_ = stamp;
  tf_frame_id_ = frame_id;
}
MovingObjectFrame::~MovingObjectFrame()
{
}

void MovingObjectFrame::addVector(const std::vector<DetectionObjectInBox>& vector)
{
  objects_detected_ = vector;
}

void MovingObjectFrame::addVector(const std::vector<TrackingObjectInBox>& vector)
{
  objects_tracked_ = vector;
}

void MovingObjectFrame::addVector(const std::vector<LocalizationObjectInBox>& vector)
{
  objects_localized_ = vector;
}

void MovingObjectFrame::mergeObjects()
{
  if (published_ || !isDataReady())
  {
    RCLCPP_WARN(node_->get_logger(), "Already published or data not ready. Do nothing");
    return;
  }

  for (std::vector<DetectionObjectInBox>::iterator it = objects_detected_.begin();
       it != objects_detected_.end(); ++it)
  {
#ifdef MO_VERBOSE
    RCLCPP_INFO(node_->get_logger(),"...ENTER Checking merging condition[probability=%f, \
    threshold=%f]...", it->object.probability, params_->posibility_threshold_);
#endif
    if (it->object.probability < params_->posibility_threshold_)
    {
      RCLCPP_INFO(node_->get_logger(), ".....Probability of one of objects does not \
match[probability=%f, threshold=%f]...",
                  it->object.probability, params_->posibility_threshold_);
      continue;
    }

    if (params_->social_filtering_enabled_ && social_filter_.size() > 0 && !isSocialObject(*it))
    {
      RCLCPP_INFO(node_->get_logger(), "......one object is not social object");
      continue;
    }

    ObjectRoi roi = it->roi;
    MovingObject moving_obj;
    TrackingObjectInBox track_obj;
    LocalizationObjectInBox loc_obj;

    bool result = findTrackingObjectByRoi(roi, track_obj);
    if (result)
    {
      result = findLocalizationObjectByRoi(roi, loc_obj);
      if (result)
      {
        RCLCPP_INFO(node_->get_logger(), "...New Object is merging...");
        moving_obj.min = loc_obj.min;
        moving_obj.max = loc_obj.max;
        moving_obj.id = track_obj.id;
        moving_obj.type = it->object.object_name;
        moving_obj.probability = it->object.probability;
        moving_obj.roi = it->roi;
        // Give a non-sense value as the original velocity.
        moving_obj.velocity.x = moving_obj.velocity.y = moving_obj.velocity.z = -9999;
        moving_objects_.push_back(moving_obj);
      }
    }

  }  // end of for(...)
}

bool MovingObjectFrame::findMovingObjectById(const int id, MovingObject& out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects)
  {
    if (t.id == id)
    {
      out = t;
      return true;
    }
  }
  return false;
}

bool MovingObjectFrame::findMovingObjectByRoi(const ObjectRoi& roi, MovingObject& out)
{
  MovingObjectVector temp_objects = moving_objects_;
  for (auto t : temp_objects)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset &&
        roi.width == t.roi.width && roi.height == t.roi.height)
    {
      out = t;
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::findTrackingObjectByRoi(const ObjectRoi& roi, TrackingObjectInBox& track)
{
  float x1 = roi.x_offset;
  float y1 = roi.y_offset;
  float width1 = roi.width;
  float height1 = roi.height;
  for (auto t : objects_tracked_) {
    float x2 = t.roi.x_offset;
    float y2 = t.roi.y_offset;
    float width2 = t.roi.width;
    float height2 = t.roi.height;
    float endx = std::max(x1 + width1, x2 + width2);
    float startx = std::min(x1, x2);
    float width = width1 + width2 - (endx - startx);
    float endy = std::max(y1 + height1, y2 + height2);
    float starty = std::min(y1, y2);
    float height = height1 + height2 - (endy - starty);
    float overlap_ratio;
    if (width <= 0 || height <= 0) {
      overlap_ratio = 0;
    } else {
      overlap_ratio = width * height /
        (width1 * height1 + width2 * height2 - width * height);
    }
    if (overlap_ratio >= params_->overlap_ratio_threshold) {
      track = t;
      RCLCPP_INFO(node_->get_logger(), "<<<<<FOUND Tracking ROI:%8d:%8d:%8d:%8d", t.roi.x_offset,
                  t.roi.y_offset, t.roi.width, t.roi.height);
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::findLocalizationObjectByRoi(const ObjectRoi& roi,
                                                    LocalizationObjectInBox& loc)
{
  for (auto t : objects_localized_)
  {
#ifdef MO_VERBOSE
    RCLCPP_INFO(node_->get_logger(),">>>>>Loc ROI:%8d:%8d:%8d:%8d", roi.x_offset,
                roi.y_offset, roi.width, roi.height);
    RCLCPP_INFO(node_->get_logger(),"<<<<<Loc ROI:%8d:%8d:%8d:%8d", t.roi.x_offset,
                t.roi.y_offset, t.roi.width, t.roi.height);
#endif
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset &&
        roi.width == t.roi.width && roi.height == t.roi.height)
    {
      loc = t;
      RCLCPP_INFO(node_->get_logger(), "<<<<<FOUND Loc ROI:%8d:%8d:%8d:%8d", t.roi.x_offset,
                  t.roi.y_offset, t.roi.width, t.roi.height);
      return true;
    }
  }

  return false;
}

bool MovingObjectFrame::publish()
{
  if (published_)
  {
    RCLCPP_INFO(node_->get_logger(), "Moving objects have been already published, do nothing");
    return false;
  }

  if (params_->moving_object_msg_enabled_)
  {
    MovingObjectMsg::SharedPtr msg = std::make_shared<MovingObjectMsg>();

    msg->header.frame_id = tf_frame_id_;
    msg->header.stamp = stamp_;
    msg->objects = moving_objects_;

    RCLCPP_INFO(node_->get_logger(), "...New Moving Object is publishing [Size of MO:%d] ...",
                msg->objects.size());
    moving_objects_pub_->publish(msg);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "moving_object_msg_enabled is set to FALSE, don't publish "
                                     "...");
  }

  setFlagPublished(true);

  return true;
}

bool MovingObjectFrame::isSocialObject(DetectionObjectInBox& ob)
{
  for (auto f : social_filter_)
  {
    if (ob.object.object_name.find(f) != std::string::npos)
    {
      return true;
    }
  }

  return false;
}

void MovingObjectFrame::setFlagPublished(bool state)
{
  published_ = state;
}
}  // namespace
