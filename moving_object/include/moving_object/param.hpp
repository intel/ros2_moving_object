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

#ifndef ICA_MOVING_OBJECT_PARAM_H
#define ICA_MOVING_OBJECT_PARAM_H

#include <string>
#include "moving_object/remap.hpp"

namespace moving_object
{
class Param
{
public:
  using Ptr = std::shared_ptr<Param>;
  using ConstPtr = std::shared_ptr<Param const>;
  friend class MovingObjectRos;
  friend class MovingObjectFrame;
  friend class MovingObjects;

  Param();
  Param(const std::string& file_path);

  bool loadParamFromYAML(const std::string& file_path);
  bool validateParam();

private:
  void init();

  bool social_filtering_enabled_;
  bool moving_object_msg_enabled_;
  double posibility_threshold_;
  int max_frames_; /**< The number of frames to be archived in memory. */
  bool velocity_enabled_;
  std::string fixed_frame_;
  double overlap_ratio_threshold_;

  /**< Object Messages, which are initialized from parameter re-configure.*/
  std::string msg_object_detection_;
  std::string msg_object_tracking_;
  std::string msg_object_localization_;
};
}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_PARAM_H
