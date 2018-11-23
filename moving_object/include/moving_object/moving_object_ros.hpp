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

#ifndef ICA_MOVING_OBJECT_ROS_H
#define ICA_MOVING_OBJECT_ROS_H
#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "moving_object/moving_object.hpp"
#include "moving_object/param.hpp"

namespace moving_object
{
/** @brief Merging camera related messages into one, with more info(currently velocity) added.
 *         When a given message is received:
 *         1. The class searches the corresponding frame by the same frame_id and stamp, if no,
 * creates a new frame.
 *         2. Add the message to the frame which filtered out in step1.
 *         3. Publish the ready frame(s).
 *         4. Clean the frames which are not in the monitoring window.
 */
class MovingObjectRos : public rclcpp::Node
{
public:
  explicit MovingObjectRos(const std::shared_ptr<Param>& param = nullptr);
  virtual ~MovingObjectRos();
  bool setParameters(const std::shared_ptr<Param> params);

private:
  void onObjectsReceived(const DetectionMsg::SharedPtr& detect, const TrackingMsg::SharedPtr& track,
                         const LocalizationMsg::SharedPtr& loc);

  std::shared_ptr<MovingObjects> frames_;
  std::shared_ptr<Param> params_;

  using FilteredDetection = message_filters::Subscriber<DetectionMsg>;
  using FilteredTracking = message_filters::Subscriber<TrackingMsg>;
  using FilteredLocalization = message_filters::Subscriber<LocalizationMsg>;
  using FilteredSync =
      message_filters::TimeSynchronizer<DetectionMsg, TrackingMsg, LocalizationMsg>;

  std::unique_ptr<FilteredDetection> f_detection_sub_;
  std::unique_ptr<FilteredTracking> f_tracking_sub_;
  std::unique_ptr<FilteredLocalization> f_localization_sub_;
  std::unique_ptr<FilteredSync> sync_sub_;

  std::mutex mutex_;
};

}  // namespace
#endif
