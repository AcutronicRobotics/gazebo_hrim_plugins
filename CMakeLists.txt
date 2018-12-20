cmake_minimum_required(VERSION 3.5)
project(gazebo_hrim_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hrim_sensor_camera_msgs REQUIRED)
find_package(hrim_composite_arm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${hrim_sensor_camera_msgs_INCLUDE_DIRS}
  ${hrim_composite_arm_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_hrim_joint_state_publisher
add_library(gazebo_hrim_joint_state_publisher SHARED
  src/gazebo_hrim_joint_state_publisher.cpp
)
ament_target_dependencies(gazebo_hrim_joint_state_publisher
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "hrim_composite_arm_msgs"
)
ament_export_libraries(gazebo_hrim_joint_state_publisher)

# gazebo_ros_camera
add_library(gazebo_hrim_camera SHARED
  src/gazebo_hrim_camera.cpp
)
ament_target_dependencies(gazebo_hrim_camera
  "gazebo_dev"
  "gazebo_ros"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  "hrim_sensor_camera_msgs"
  "std_msgs"
)
target_link_libraries(gazebo_hrim_camera
  CameraPlugin
)
ament_export_libraries(gazebo_hrim_camera)

# gazebo_hrim_imu_sensor
add_library(gazebo_hrim_imu_sensor SHARED
  src/gazebo_hrim_imu_sensor.cpp
)
ament_target_dependencies(gazebo_hrim_imu_sensor
  "gazebo_ros"
  "sensor_msgs"
  "gazebo_dev"
)
ament_export_libraries(gazebo_hrim_imu_sensor)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_hrim_camera
    gazebo_hrim_imu_sensor
    gazebo_hrim_joint_state_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
