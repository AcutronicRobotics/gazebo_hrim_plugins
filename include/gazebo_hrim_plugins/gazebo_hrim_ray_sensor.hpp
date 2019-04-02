// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
#define GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_

// #include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <hrim_sensor_lidar_msgs/msg/lidar_scan.hpp>
#include <hrim_sensor_rangefinder_msgs/msg/distance.hpp>
#include <hrim_sensor_rangefinder_srvs/srv/specs_rangefinder.hpp>
#include <hrim_sensor_3dcameratof_msgs/msg/point_field.hpp>
#include <hrim_sensor_3dcameratof_msgs/msg/point_cloud.hpp>

#include <memory>

namespace gazebo_hrim_plugins
{

class GazeboRosRaySensorPrivate;

/// Plugin to attach to a gazebo ray or gpu_ray sensor and publish its data to ROS
/**
  Example SDF:
  \code{.xml}
    <plugin name="my_ray_sensor_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <!-- Configure namespace -->
        <namespace>/ray</namespace>
        <argument>~/outlaser:=laser_scan</argument>
        <argument>~/outspecs:=specs</argument>
        <argument>~/outrange:=distance</argument>
      </ros>
      <!-- Clip intensity values so all are above 100, optional -->
      <min_intensity>100.0</min_intensity>
      <!-- Frame id for header of output, defaults to sensor's parent link name -->
      <frame_name>ray_link</frame_name>
    </plugin>
  \endcode
*/
class GazeboRosRaySensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosRaySensor();

  /// \brief Destructor
  virtual ~GazeboRosRaySensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<GazeboRosRaySensorPrivate> impl_;
};

}  // namespace gazebo_hrim_plugins

#endif  // GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
