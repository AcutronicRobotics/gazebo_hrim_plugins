// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_
#define GAZEBO_HRIM_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_hrim_plugins
{
class GazeboRosFTSensorPrivate;

/// Publish the state of a joint child's force and torque in the simulation to a given ROS topic.
/**

  Valid joint types are limited to:
  - revolute
  - prismatic
  - fixed

  Example Usage:
  \code{.xml}
  <plugin name="my_ft"
    filename="libgazebo_hrim_ft_sensor.so">
    <ros>
      <namespace>/demo</namespace>
      <argument>~/out:=reading</argument>
    </ros>
    <update_rate>100</update_rate>
    <joint_name>pendulum</joint_name>
  </plugin>
  \endcode

  <provide_feedback> must be set to true on the selected joint:
  \code{.xml}
  <joint ...>
    ...
    <physics>
      <provide_feedback>true</provide_feedback>
    </physics>
  </joint>
  \endcode
*/
class GazeboRosFTSensor : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosFTSensor();

  /// Destructor
  ~GazeboRosFTSensor();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Callback to be called at every simulation iteration.
  /// Private data pointer
  std::unique_ptr<GazeboRosFTSensorPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_
