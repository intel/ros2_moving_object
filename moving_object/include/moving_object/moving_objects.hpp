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
#ifndef ICA_MOVING_OBJECT_H
#define ICA_MOVING_OBJECT_H

#include <vector>
#include <math.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_analytics_msgs/msg/tracked_objects.hpp>
#include <object_analytics_msgs/msg/objects_in_boxes3_d.hpp>
#include "moving_object/object_frame.hpp"
#include "moving_object/param.hpp"
#include "moving_object/remap.hpp"

namespace moving_object
{
constexpr int kDefaultMaxFrames = 20;

class MovingObjects
{
public:
  MovingObjects(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<Param>& param);
  virtual ~MovingObjects();

  /** @brief Callback function for message filters interface.
     *  @param[in] detect   detection messages received from vision object component.
     *  @param[in] track    tracking messages received from vision object component.
     *  @param[in] loc      localization messages received from vision object component.
     */
  void processFrame(const object_msgs::msg::ObjectsInBoxes::SharedPtr& detect,
                    const object_analytics_msgs::msg::TrackedObjects::SharedPtr& track,
                    const object_analytics_msgs::msg::ObjectsInBoxes3D::SharedPtr& loc);

  /** @brief Calculates velocity for a given Object Frame.
   *  @param[in+out] frame The frame to be calculated.
   */
  void calcVelocity(std::shared_ptr<MovingObjectFrame>& frame);

  /** @brief Search and return the instance of a frame by the given frame_id and time stamp.
   *         If no cached instance, create a new one and return it.
   *  @param[in] stamp    The time stamp when the frame is taken, which is used to search the frame.
   *  @param[in] frame_id The id the frame, which is used to search the frame.
   *  @return the pointer to the frame instance.
   */
  std::shared_ptr<MovingObjectFrame> getInstance(const builtin_interfaces::msg::Time stamp,
                                                 const std::string frame_id);

  /** @brief Find and return the object frame by the given time stamp and frame id.
   *  @param[in]  stamp     The time stamp when the frame is taken, which is used to search the
   * frame.
   *  @param[in]  frame_id  The id the frame, which is used to search the frame.
   *  @return     the shared pointer of found MovingObjectFrame if it is found, otherwise nullptr.
   */
  std::shared_ptr<MovingObjectFrame> findObjectFrame(const builtin_interfaces::msg::Time stamp,
                                                     const std::string frame_id);

  /** @brief Clean the cached frames and remove the old ones if the size of frames is over the
   * threshold. */
  void clearOldFrames();

private:
  double durationBtwFrames(MovingObjectFrame& first, MovingObjectFrame& second);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::shared_ptr<MovingObjectFrame> > frames_;

  std::shared_ptr<Param> params_;
};

}  // namespace
#endif
