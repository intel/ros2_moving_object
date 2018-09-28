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
#include <gtest/gtest.h>
#include <cassert>
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <moving_object_msgs/msg/moving_object.hpp>
#include <moving_object_msgs/msg/moving_objects_in_frame.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "moving_object/object_frame.hpp"
#include "moving_object/remap.hpp"
#include "moving_object/param.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "moving_object/moving_objects.hpp"
#include "unittest_util.hpp"

using namespace object_msgs::msg;
using namespace object_analytics_msgs::msg;
using namespace moving_object;
using namespace std;

#define MAX_FRAMES 30 // params_->max_frames_

TEST(UnitTestMovingObject, processFrame_calcVelocity)
{
  auto node = rclcpp::Node::make_shared("test_node1");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjects movingobjects(node, param);
  
  auto detect = std::make_shared<DetectionMsg>();
  auto track = std::make_shared<TrackingMsg>();
  auto loc = std::make_shared<LocalizationMsg>();
  detect->objects_vector.clear();
  track->tracked_objects.clear();
  loc->objects_in_boxes.clear();

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 200000000;
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.5f);
  detect->header.stamp = stamp;
  detect->header.frame_id = "test1";
  detect->objects_vector.push_back(DetObj_inbox_first);
  TrackingObjectInBox TraObj_inbox_first;
  TraObj_inbox_first.id = 0;
  TraObj_inbox_first.object = getObject("person", 0.5f);
  TraObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  track->tracked_objects.push_back(TraObj_inbox_first);
  LocalizationObjectInBox LocObj_inbox_first;
  LocObj_inbox_first.object = getObject("person", 0.5f);
  LocObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  LocObj_inbox_first.min.x = 100;
  LocObj_inbox_first.min.y = 50;
  LocObj_inbox_first.min.z = 200;
  LocObj_inbox_first.max.x = 200;
  LocObj_inbox_first.max.y = 150;
  LocObj_inbox_first.max.z = 320;
  loc->objects_in_boxes.push_back(LocObj_inbox_first);
  movingobjects.processFrame(detect, track, loc);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.x, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.y, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.z, -9999);

  loc->objects_in_boxes[0].min.x = 110;
  loc->objects_in_boxes[0].min.y = 55;
  loc->objects_in_boxes[0].min.z = 220;
  loc->objects_in_boxes[0].max.x = 210;
  loc->objects_in_boxes[0].max.y = 160;
  loc->objects_in_boxes[0].max.z = 330;
  stamp.sec = 100000000;
  stamp.nanosec = 250000000;
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(60, 60, 110, 110, "person", 0.5f);
  detect->header.stamp = stamp;
  detect->header.frame_id = "test1";
  detect->objects_vector.push_back(DetObj_inbox_second);
  TrackingObjectInBox TraObj_inbox_second;
  TraObj_inbox_second.id = 1;
  TraObj_inbox_second.object = getObject("person", 0.5f);
  TraObj_inbox_second.roi = getRoi(60, 60, 110, 110);
  track->tracked_objects.push_back(TraObj_inbox_second);
  LocalizationObjectInBox LocObj_inbox_second;
  LocObj_inbox_second.object = getObject("person", 0.5f);
  LocObj_inbox_second.roi = getRoi(60, 60, 110, 110);
  LocObj_inbox_second.min.x = 120;
  LocObj_inbox_second.min.y = 60;
  LocObj_inbox_second.min.z = 240;
  LocObj_inbox_second.max.x = 230;
  LocObj_inbox_second.max.y = 180;
  LocObj_inbox_second.max.z = 350;
  loc->objects_in_boxes.push_back(LocObj_inbox_second);
  movingobjects.processFrame(detect, track, loc);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.x, (double)((110 + 210) - (100 + 200)) * 100 / ((25 - 20) * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.y, (double)((55 + 160) - (50 + 150)) * 100 / ((25 - 20) * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.z, (double)((220 + 330) - (200 + 320)) * 100 / ((25 - 20) * 2), 0.000001);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.x, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.y, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.z, -9999);

  loc->objects_in_boxes[0].min.x = 115;
  loc->objects_in_boxes[0].min.y = 65;
  loc->objects_in_boxes[0].min.z = 250;
  loc->objects_in_boxes[0].max.x = 235;
  loc->objects_in_boxes[0].max.y = 185;
  loc->objects_in_boxes[0].max.z = 355;
  loc->objects_in_boxes[1].min.x = 130;
  loc->objects_in_boxes[1].min.y = 70;
  loc->objects_in_boxes[1].min.z = 260;
  loc->objects_in_boxes[1].max.x = 240;
  loc->objects_in_boxes[1].max.y = 190;
  loc->objects_in_boxes[1].max.z = 360;
  stamp.sec = 100000000;
  stamp.nanosec = 320000000;
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(70, 70, 120, 120, "person", 0.5f);
  detect->header.stamp = stamp;
  detect->header.frame_id = "test1";
  detect->objects_vector.push_back(DetObj_inbox_third);
  TrackingObjectInBox TraObj_inbox_third;
  TraObj_inbox_third.id = 2;
  TraObj_inbox_third.object = getObject("person", 0.5f);
  TraObj_inbox_third.roi = getRoi(70, 70, 120, 120);
  track->tracked_objects.push_back(TraObj_inbox_third);
  LocalizationObjectInBox LocObj_inbox_third;
  LocObj_inbox_third.object = getObject("person", 0.5f);
  LocObj_inbox_third.roi = getRoi(70, 70, 120, 120);
  LocObj_inbox_third.min.x = 150;
  LocObj_inbox_third.min.y = 75;
  LocObj_inbox_third.min.z = 270;
  LocObj_inbox_third.max.x = 250;
  LocObj_inbox_third.max.y = 210;
  LocObj_inbox_third.max.z = 375;
  loc->objects_in_boxes.push_back(LocObj_inbox_third);
  movingobjects.processFrame(detect, track, loc);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.x, (double)((115 + 235) - (100 + 200)) * 100 / ((32 - 20) * 2 * 2) + (double)((115 + 235) - (110 + 210)) * 100 / ((32 - 25) * 2 * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.y, (double)((65 + 185) - (50 + 150)) * 100 / ((32 - 20) * 2 * 2) + (double)((65 + 185) - (55 + 160)) * 100 / ((32 - 25) * 2 * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[0].velocity.z, (double)((250 + 355) - (200 + 320)) * 100 / ((32 - 20) * 2 * 2) + (double)((250 + 355) - (220 + 330)) * 100 / ((32 - 25) * 2 * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.x, (double)((130 + 240) - (120 + 230)) * 100 / ((32 - 25) * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.y, (double)((70 + 190) - (60 + 180)) * 100 / ((32 - 25) * 2), 0.000001);
  EXPECT_NEAR(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[1].velocity.z, (double)((260 + 360) - (240 + 350)) * 100 / ((32 - 25) * 2), 0.000001);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[2].velocity.x, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[2].velocity.y, -9999);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test1")->getMovingObjects()[2].velocity.z, -9999);
}

TEST(UnitTestMovingObject, findObjectFrame)
{
  builtin_interfaces::msg::Time stamp;
  auto node = rclcpp::Node::make_shared("test_node2");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjects movingobjects(node, param);
  
  stamp.sec = 100000000;
  stamp.nanosec = 200000000;
  movingobjects.getInstance(stamp, "test2");
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getStamp(), stamp);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getTfFrameId(), "test2");
  stamp.sec = 100000000;
  stamp.nanosec = 250000000;
  movingobjects.getInstance(stamp, "test2");
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getStamp(), stamp);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getTfFrameId(), "test2");
  stamp.sec = 100000000;
  stamp.nanosec = 320000000;
  movingobjects.getInstance(stamp, "test2");
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getStamp(), stamp);
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test2")->getTfFrameId(), "test2");
}

TEST(UnitTestMovingObject, clearOldFrames)
{
  builtin_interfaces::msg::Time stamp;
  auto node = rclcpp::Node::make_shared("test_node3");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjects movingobjects(node, param);

  stamp.sec = 100000000;
  stamp.nanosec = 300000000;
  for(int i = 0;i < MAX_FRAMES + 1;i++)
  {
    movingobjects.getInstance(stamp, "test3");
    EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test3")->getStamp(), stamp);
    stamp.nanosec += 1;
  }
  stamp.sec = 100000000;
  stamp.nanosec = 300000000;
  movingobjects.clearOldFrames();
  EXPECT_EQ(movingobjects.findObjectFrame(stamp, "test3"), nullptr);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
