find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(object_msgs REQUIRED)
find_package(object_analytics_msgs REQUIRED)
find_package(moving_object_msgs REQUIRED)

ament_lint_auto_find_test_dependencies()
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${moving_object_INCLUDE_DIRS})

ament_add_gtest(unittest_moving_object unittest_moving_object.cpp ../src/moving_object.cpp ../src/object_frame.cpp ../src/param.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
ament_target_dependencies(unittest_moving_object
  "rclcpp"
  "sensor_msgs"
  "object_msgs"
  "object_analytics_msgs"
  "moving_object_msgs"
)
if(TARGET unittest_moving_object)
  target_link_libraries(unittest_moving_object ${GTEST_LIBRARIES} rviz-yaml-cpp)
endif()

ament_add_gtest(unittest_object_frame unittest_object_frame.cpp ../src/object_frame.cpp ../src/param.cpp unittest_util.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
ament_target_dependencies(unittest_object_frame
  "rclcpp"
  "sensor_msgs"
  "object_msgs"
  "object_analytics_msgs"
  "moving_object_msgs"
)
if(TARGET unittest_object_frame)
  target_link_libraries(unittest_object_frame ${GTEST_LIBRARIES} rviz-yaml-cpp)
endif()
