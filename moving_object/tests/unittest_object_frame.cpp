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
#include "unittest_util.hpp"

using namespace object_msgs::msg;
using namespace object_analytics_msgs::msg;
using namespace moving_object;

TEST(UnitTestMovingObjectFrame, MovingObjectFrame)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node1");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test1", node, param);
  
  EXPECT_EQ(frame.getTfFrameId(), "test1");
  EXPECT_EQ(frame.getStamp().sec, 100000000);
  EXPECT_EQ(frame.getStamp().nanosec, (unsigned int)111111111);
}

TEST(UnitTestMovingObjectFrame, isSocialObject)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node2");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test2", node, param);

  std::vector<DetectionObjectInBox> Detection_vector;
  Detection_vector.clear();
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  Detection_vector.push_back(DetObj_inbox_first);
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  Detection_vector.push_back(DetObj_inbox_second);
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  Detection_vector.push_back(DetObj_inbox_third);
  EXPECT_EQ(Detection_vector.size(), static_cast<size_t>(3));
  frame.addVector(Detection_vector);

  EXPECT_EQ(frame.isSocialObject(Detection_vector[0]), true);
  EXPECT_EQ(frame.isSocialObject(Detection_vector[1]), false);
}

TEST(UnitTestMovingObjectFrame, getMovingObjects)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node3");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test3", node, param);

  std::vector<DetectionObjectInBox> Detection_vector;
  Detection_vector.clear();
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  Detection_vector.push_back(DetObj_inbox_first);
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  Detection_vector.push_back(DetObj_inbox_second);
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  Detection_vector.push_back(DetObj_inbox_third);
  EXPECT_EQ(Detection_vector.size(), static_cast<size_t>(3));
  frame.addVector(Detection_vector);
  
  std::vector<TrackingObjectInBox> Tracking_vector;
  Tracking_vector.clear();
  TrackingObjectInBox TraObj_inbox;
  TraObj_inbox.id = 0;
  TraObj_inbox.object = getObject("person", 0.8f);
  TraObj_inbox.roi = getRoi(50, 50, 100, 100);
  Tracking_vector.push_back(TraObj_inbox);
  EXPECT_EQ(Tracking_vector.size(), static_cast<size_t>(1));
  frame.addVector(Tracking_vector);
  
  std::vector<LocalizationObjectInBox> Localization_vector;
  Localization_vector.clear();
  LocalizationObjectInBox LocObj_inbox_first;
  LocObj_inbox_first.object = getObject("person", 0.8f);
  LocObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  LocObj_inbox_first.min.x = 100;
  LocObj_inbox_first.min.y = 50;
  LocObj_inbox_first.min.z = 200;
  LocObj_inbox_first.max.x = 200;
  LocObj_inbox_first.max.y = 150;
  LocObj_inbox_first.max.z = 320;
  Localization_vector.push_back(LocObj_inbox_first);
  LocalizationObjectInBox LocObj_inbox_second;
  LocObj_inbox_second.object = getObject("chair", 0.9f);
  LocObj_inbox_second.roi = getRoi(100, 100, 50, 50);
  LocObj_inbox_second.min.x = 50;
  LocObj_inbox_second.min.y = 80;
  LocObj_inbox_second.min.z = 300;
  LocObj_inbox_second.max.x = 160;
  LocObj_inbox_second.max.y = 110;
  LocObj_inbox_second.max.z = 400;
  Localization_vector.push_back(LocObj_inbox_second);
  EXPECT_EQ(Localization_vector.size(), static_cast<size_t>(2));
  frame.addVector(Localization_vector);

  EXPECT_EQ(frame.isDataReady(), true);

  frame.mergeObjects(); 
  std::vector<moving_object_msgs::msg::MovingObject> Moving_vector;
  Moving_vector.clear();
  Moving_vector = frame.getMovingObjects();
  
  EXPECT_EQ(Moving_vector.size(), static_cast<size_t>(1));
  EXPECT_EQ(Moving_vector[0].min, Localization_vector[0].min);
  EXPECT_EQ(Moving_vector[0].max, Localization_vector[0].max);
  EXPECT_EQ(Moving_vector[0].id, 0);
  EXPECT_EQ(Moving_vector[0].type, "person");
  EXPECT_NEAR(Moving_vector[0].probability, 0.5, 0.000001);
  EXPECT_EQ(Moving_vector[0].roi, Localization_vector[0].roi);
  EXPECT_EQ(Moving_vector[0].velocity.x, -9999);
  EXPECT_EQ(Moving_vector[0].velocity.y, -9999);
  EXPECT_EQ(Moving_vector[0].velocity.z, -9999);
}

TEST(UnitTestMovingObjectFrame, findMovingObjectByRoi)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node4");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test4", node, param);

  std::vector<DetectionObjectInBox> Detection_vector;
  Detection_vector.clear();
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  Detection_vector.push_back(DetObj_inbox_first);
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  Detection_vector.push_back(DetObj_inbox_second);
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  Detection_vector.push_back(DetObj_inbox_third);
  EXPECT_EQ(Detection_vector.size(), static_cast<size_t>(3));
  frame.addVector(Detection_vector);
  
  std::vector<TrackingObjectInBox> Tracking_vector;
  Tracking_vector.clear();
  TrackingObjectInBox TraObj_inbox;
  TraObj_inbox.id = 0;
  TraObj_inbox.object = getObject("person", 0.8f);
  TraObj_inbox.roi = getRoi(50, 50, 100, 100);
  Tracking_vector.push_back(TraObj_inbox);
  EXPECT_EQ(Tracking_vector.size(), static_cast<size_t>(1));
  frame.addVector(Tracking_vector);
  
  std::vector<LocalizationObjectInBox> Localization_vector;
  Localization_vector.clear();
  LocalizationObjectInBox LocObj_inbox_first;
  LocObj_inbox_first.object = getObject("person", 0.8f);
  LocObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  LocObj_inbox_first.min.x = 100;
  LocObj_inbox_first.min.y = 50;
  LocObj_inbox_first.min.z = 200;
  LocObj_inbox_first.max.x = 200;
  LocObj_inbox_first.max.y = 150;
  LocObj_inbox_first.max.z = 320;
  Localization_vector.push_back(LocObj_inbox_first);
  LocalizationObjectInBox LocObj_inbox_second;
  LocObj_inbox_second.object = getObject("chair", 0.9f);
  LocObj_inbox_second.roi = getRoi(100, 100, 50, 50);
  LocObj_inbox_second.min.x = 50;
  LocObj_inbox_second.min.y = 80;
  LocObj_inbox_second.min.z = 300;
  LocObj_inbox_second.max.x = 160;
  LocObj_inbox_second.max.y = 110;
  LocObj_inbox_second.max.z = 400;
  Localization_vector.push_back(LocObj_inbox_second);
  EXPECT_EQ(Localization_vector.size(), static_cast<size_t>(2));
  frame.addVector(Localization_vector);

  EXPECT_EQ(frame.isDataReady(), true);

  frame.mergeObjects(); 
  moving_object_msgs::msg::MovingObject Moving_object;
  EXPECT_EQ(frame.findMovingObjectByRoi(frame.getMovingObjects()[0].roi, Moving_object), true);
  EXPECT_EQ(Moving_object.min, Localization_vector[0].min);
  EXPECT_EQ(Moving_object.max, Localization_vector[0].max);
  EXPECT_EQ(Moving_object.id, 0);
  EXPECT_EQ(Moving_object.type, "person");
  EXPECT_NEAR(Moving_object.probability, 0.5, 0.000001);
  EXPECT_EQ(Moving_object.velocity.x, -9999);
  EXPECT_EQ(Moving_object.velocity.y, -9999);
  EXPECT_EQ(Moving_object.velocity.z, -9999);
}

TEST(UnitTestMovingObjectFrame, findMovingObjectById)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node5");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test5", node, param);

  std::vector<DetectionObjectInBox> Detection_vector;
  Detection_vector.clear();
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  Detection_vector.push_back(DetObj_inbox_first);
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  Detection_vector.push_back(DetObj_inbox_second);
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  Detection_vector.push_back(DetObj_inbox_third);
  EXPECT_EQ(Detection_vector.size(), static_cast<size_t>(3));
  frame.addVector(Detection_vector);
  
  std::vector<TrackingObjectInBox> Tracking_vector;
  Tracking_vector.clear();
  TrackingObjectInBox TraObj_inbox;
  TraObj_inbox.id = 0;
  TraObj_inbox.object = getObject("person", 0.8f);
  TraObj_inbox.roi = getRoi(50, 50, 100, 100);
  Tracking_vector.push_back(TraObj_inbox);
  EXPECT_EQ(Tracking_vector.size(), static_cast<size_t>(1));
  frame.addVector(Tracking_vector);
  
  std::vector<LocalizationObjectInBox> Localization_vector;
  Localization_vector.clear();
  LocalizationObjectInBox LocObj_inbox_first;
  LocObj_inbox_first.object = getObject("person", 0.8f);
  LocObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  LocObj_inbox_first.min.x = 100;
  LocObj_inbox_first.min.y = 50;
  LocObj_inbox_first.min.z = 200;
  LocObj_inbox_first.max.x = 200;
  LocObj_inbox_first.max.y = 150;
  LocObj_inbox_first.max.z = 320;
  Localization_vector.push_back(LocObj_inbox_first);
  LocalizationObjectInBox LocObj_inbox_second;
  LocObj_inbox_second.object = getObject("chair", 0.9f);
  LocObj_inbox_second.roi = getRoi(100, 100, 50, 50);
  LocObj_inbox_second.min.x = 50;
  LocObj_inbox_second.min.y = 80;
  LocObj_inbox_second.min.z = 300;
  LocObj_inbox_second.max.x = 160;
  LocObj_inbox_second.max.y = 110;
  LocObj_inbox_second.max.z = 400;
  Localization_vector.push_back(LocObj_inbox_second);
  EXPECT_EQ(Localization_vector.size(), static_cast<size_t>(2));
  frame.addVector(Localization_vector);

  EXPECT_EQ(frame.isDataReady(), true);

  frame.mergeObjects(); 
  moving_object_msgs::msg::MovingObject Moving_object;
  EXPECT_EQ(frame.findMovingObjectById(frame.getMovingObjects()[0].id, Moving_object), true);
  EXPECT_EQ(Moving_object.min, Localization_vector[0].min);
  EXPECT_EQ(Moving_object.max, Localization_vector[0].max);
  EXPECT_EQ(Moving_object.roi, Localization_vector[0].roi);
  EXPECT_EQ(Moving_object.type, "person");
  EXPECT_NEAR(Moving_object.probability, 0.5, 0.000001);
  EXPECT_EQ(Moving_object.velocity.x, -9999);
  EXPECT_EQ(Moving_object.velocity.y, -9999);
  EXPECT_EQ(Moving_object.velocity.z, -9999);
}

TEST(UnitTestMovingObjectFrame, getCentroid)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100000000;
  stamp.nanosec = 111111111;
  auto node = rclcpp::Node::make_shared("test_node6");
  Param* p = new Param;
  std::shared_ptr<Param> param(p);
  MovingObjectFrame frame(stamp, "test6", node, param);

  std::vector<DetectionObjectInBox> Detection_vector;
  Detection_vector.clear();
  DetectionObjectInBox DetObj_inbox_first = getObjectInBox(50, 50, 100, 100, "person", 0.8f);
  Detection_vector.push_back(DetObj_inbox_first);
  DetectionObjectInBox DetObj_inbox_second = getObjectInBox(100, 100, 50, 50, "chair", 0.9f);
  Detection_vector.push_back(DetObj_inbox_second);
  DetectionObjectInBox DetObj_inbox_third = getObjectInBox(320, 480, 1, 1, "key", 0.1f);
  Detection_vector.push_back(DetObj_inbox_third);
  EXPECT_EQ(Detection_vector.size(), static_cast<size_t>(3));
  frame.addVector(Detection_vector);
  
  std::vector<TrackingObjectInBox> Tracking_vector;
  Tracking_vector.clear();
  TrackingObjectInBox TraObj_inbox;
  TraObj_inbox.id = 0;
  TraObj_inbox.object = getObject("person", 0.8f);
  TraObj_inbox.roi = getRoi(50, 50, 100, 100);
  Tracking_vector.push_back(TraObj_inbox);
  EXPECT_EQ(Tracking_vector.size(), static_cast<size_t>(1));
  frame.addVector(Tracking_vector);
  
  std::vector<LocalizationObjectInBox> Localization_vector;
  Localization_vector.clear();
  LocalizationObjectInBox LocObj_inbox_first;
  LocObj_inbox_first.object = getObject("person", 0.8f);
  LocObj_inbox_first.roi = getRoi(50, 50, 100, 100);
  LocObj_inbox_first.min.x = 100;
  LocObj_inbox_first.min.y = 50;
  LocObj_inbox_first.min.z = 200;
  LocObj_inbox_first.max.x = 200;
  LocObj_inbox_first.max.y = 150;
  LocObj_inbox_first.max.z = 320;
  Localization_vector.push_back(LocObj_inbox_first);
  LocalizationObjectInBox LocObj_inbox_second;
  LocObj_inbox_second.object = getObject("chair", 0.9f);
  LocObj_inbox_second.roi = getRoi(100, 100, 50, 50);
  LocObj_inbox_second.min.x = 50;
  LocObj_inbox_second.min.y = 80;
  LocObj_inbox_second.min.z = 300;
  LocObj_inbox_second.max.x = 160;
  LocObj_inbox_second.max.y = 110;
  LocObj_inbox_second.max.z = 400;
  Localization_vector.push_back(LocObj_inbox_second);
  EXPECT_EQ(Localization_vector.size(), static_cast<size_t>(2));
  frame.addVector(Localization_vector);
  
  EXPECT_EQ(frame.isDataReady(), true);

  frame.mergeObjects(); 
  geometry_msgs::msg::Point32 Point_centroid;
  Point_centroid = frame.getCentroid(frame.getMovingObjects()[0]);
  EXPECT_EQ(Point_centroid.x, (100 + 200) / 2);
  EXPECT_EQ(Point_centroid.y, (50 + 150) / 2);
  EXPECT_EQ(Point_centroid.z, (200 + 320) / 2);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
