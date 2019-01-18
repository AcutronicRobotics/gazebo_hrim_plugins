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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_hrim_plugins/gazebo_hrim_ft_sensor.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hrim_sensor_forcetorque_msgs/msg/force_torque.hpp>
#include <hrim_sensor_forcetorque_msgs/msg/specs_force_axis.hpp>
#include <hrim_sensor_forcetorque_msgs/msg/specs_torque_axis.hpp>

#include <memory>
#include <string>

namespace gazebo_hrim_plugins
{
class GazeboRosFTSensorPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// ForceTorque publisher.
  rclcpp::Publisher<hrim_sensor_forcetorque_msgs::msg::ForceTorque>::SharedPtr pub_;

  /// ForceTorque message to be published
  std::shared_ptr<hrim_sensor_forcetorque_msgs::msg::ForceTorque> msg_;

  /// Joint being tracked.
  gazebo::physics::JointPtr joint_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosFTSensor::GazeboRosFTSensor()
: impl_(std::make_unique<GazeboRosFTSensorPrivate>())
{
}

GazeboRosFTSensor::~GazeboRosFTSensor()
{
}

void GazeboRosFTSensor::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Joint
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Plugin missing <joint_name>");
    impl_->ros_node_.reset();
    return;
  }

  impl_->joint_ = NULL;

  sdf::ElementPtr joint_elem = sdf->GetElement("joint_name");

  auto joint_name = joint_elem->Get<std::string>();

  auto joint = model->GetJoint(joint_name);
  if (!joint) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
  } else {
    impl_->joint_ = joint;
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Going to publish force & torque of child of joint [%s]",
      joint_name.c_str() );
  }

  joint_elem = joint_elem->GetNextElement("joint_name");

  if (!impl_->joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "No joint found.");
    impl_->ros_node_.reset();
    return;
  }

  // Update rate
  double update_rate = 100.0;
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f",
      update_rate);
  } else {
    update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->last_update_time_ = model->GetWorld()->SimTime();

  // Initialize message
  impl_->msg_ = std::make_shared<hrim_sensor_forcetorque_msgs::msg::ForceTorque>();

  // Fill message structure
  impl_->msg_->header.frame_id = impl_->joint_->GetChild()->GetName().c_str();
  impl_->msg_->axis_forces[0].axis = hrim_sensor_forcetorque_msgs::msg::SpecsForceAxis::FORCE_AXIS_X;
  impl_->msg_->axis_forces[1].axis = hrim_sensor_forcetorque_msgs::msg::SpecsForceAxis::FORCE_AXIS_Y;
  impl_->msg_->axis_forces[2].axis = hrim_sensor_forcetorque_msgs::msg::SpecsForceAxis::FORCE_AXIS_Z;
  impl_->msg_->axis_torques[0].axis = hrim_sensor_forcetorque_msgs::msg::SpecsTorqueAxis::TORQUE_AXIS_X;
  impl_->msg_->axis_torques[1].axis = hrim_sensor_forcetorque_msgs::msg::SpecsTorqueAxis::TORQUE_AXIS_Y;
  impl_->msg_->axis_torques[2].axis = hrim_sensor_forcetorque_msgs::msg::SpecsTorqueAxis::TORQUE_AXIS_Z;

  // ForceTorque publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<hrim_sensor_forcetorque_msgs::msg::ForceTorque>("~/out");

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosFTSensorPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosFTSensorPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  // Get wrench data
  auto wrench = joint_->GetForceTorque(1);

  // Populate message
  msg_->header.stamp.sec = int(current_time.sec);
  msg_->header.stamp.nanosec = uint(current_time.nsec);
  msg_->axis_forces[0].force = wrench.body2Force.X();
  msg_->axis_forces[1].force = wrench.body2Force.Y();
  msg_->axis_forces[2].force = wrench.body2Force.Z();
  msg_->axis_torques[0].torque = wrench.body2Torque.X();
  msg_->axis_torques[1].torque = wrench.body2Torque.Y();
  msg_->axis_torques[2].torque = wrench.body2Torque.Z();

  // Publish
  pub_->publish(msg_);

  // Update time
  last_update_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosFTSensor)
}  // namespace gazebo_plugins
