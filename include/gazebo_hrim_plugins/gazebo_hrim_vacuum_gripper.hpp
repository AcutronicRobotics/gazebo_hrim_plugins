/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_EXTENDER_HPP_
#define GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_EXTENDER_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_hrim_plugins
{
class GazeboRosVacuumGripperPrivate;

/// Enable control of a vacuum gripper through HRIM interfaces
/**

  Example Usage:
  \code{.xml}
  <model>
    ...
    <plugin name='gripper' filename='libgazebo_hrim_vacuum_gripper.so'>
      <ros>
        <namespace>/demo</namespace>
        <argument>~/cmd:=goalpro</argument>
        <argument>~/state:=state</argument>
      </ros>
      <vacuum_force>5</vacuum_force>
      <max_vacuum_force>50</max_vacuum_force>
      <gripper_link>gripper</gripper_link>
    </plugin>
    ...
  </model>
  \endcode

*/
class GazeboRosVacuumGripper : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosVacuumGripper();

  /// Destructor
  ~GazeboRosVacuumGripper();

protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Callback to be called at every simulation iteration.
  /// Private data pointer
  std::unique_ptr<GazeboRosVacuumGripperPrivate> impl_;
};
}  // namespace gazebo_hrim_plugins
#endif  // GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_EXTENDER_HPP_
