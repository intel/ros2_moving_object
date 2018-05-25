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

#pragma once
#ifndef ICA_REMAP_H
#define ICA_REMAP_H

#include <vector>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include <moving_object_msgs/msg/moving_object.hpp>
#include <moving_object_msgs/msg/moving_objects_in_frame.hpp>

namespace moving_object
{
using DetectionObject = object_msgs::msg::Object;
using DetectionObjectInBox = object_msgs::msg::ObjectInBox;
using TrackingObjectInBox = object_analytics_msgs::msg::TrackedObject;
using LocalizationObjectInBox = object_analytics_msgs::msg::ObjectInBox3D;
using MovingObject = moving_object_msgs::msg::MovingObject;

using DetectionMsg = object_msgs::msg::ObjectsInBoxes;
using TrackingMsg = object_analytics_msgs::msg::TrackedObjects;
using LocalizationMsg = object_analytics_msgs::msg::ObjectsInBoxes3D;
using MovingObjectMsg = moving_object_msgs::msg::MovingObjectsInFrame;

using DetectionVector = std::vector<DetectionObjectInBox>;
using TrackingVector = std::vector<TrackingObjectInBox>;
using LocalizationVector = std::vector<LocalizationObjectInBox>;
using MovingObjectVector = std::vector<MovingObject>;

using ObjectRoi = sensor_msgs::msg::RegionOfInterest;

#define MO_VERBOSE
#ifdef MO_VERBOSE
#define MO_VERBOSE_INFO(args) RCLCPP_INFO(args)
#else
#define MO_VERBOSE_INFO(args)
#endif

/**< Posibility Threshold, only objects with equal or higher posibility pass to MO.*/
constexpr double kObjectFiltering_PosibilityThreshold = 0.2;

/**< Enable or not type filtering. */
constexpr bool kObjectFiltering_TypeFiltering_Enabling = false;

/**< If type filtering enabled, only object with the listed types pass to MO. */
//constexpr char kObjectFiltering_TypeFiltering_Types[] = "person";
const std::vector<std::string> kObjectFiltering_TypeFiltering_Types = {"person"};

/**< The maximum number of frames to be archived in moving object node. */
constexpr int kFrameCache_Size = 30;

/**< Enable or not velocity calculation. */
constexpr bool kVelocityCalculation_Enabling = false;

/**< The frame name used for coordination transform during velocity calculation. */
constexpr char kVelocityCalculation_FixedFrame[] = "/map";

/**< The default message name for object detection */
constexpr char kMessage_ObjectDetection[] = "/movidius_ncs_stream/detected_objects";

/**< The default message name for object tracking */
constexpr char kMessage_ObjectTracking[] = "/object_analytics/tracking";

/**< The default message name for object localization */
constexpr char kMessage_ObjectLocalization[] = "/object_analytics/localization";

}  // namespace
#endif
