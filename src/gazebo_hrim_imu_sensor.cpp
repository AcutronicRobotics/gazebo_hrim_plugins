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


#include <gazebo_hrim_plugins/gazebo_hrim_imu_sensor.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <hrim_sensor_imu_msgs/msg/imu.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_hrim_plugins
{

class GazeboRosImuSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for imu message
  rclcpp::Publisher<hrim_sensor_imu_msgs::msg::IMU>::SharedPtr pub_;
  /// IMU message modified each update
  hrim_sensor_imu_msgs::msg::IMU::SharedPtr msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest imu data to ROS
  void OnUpdate();
};

GazeboRosImuSensor::GazeboRosImuSensor()
: impl_(std::make_unique<GazeboRosImuSensorPrivate>())
{
}

GazeboRosImuSensor::~GazeboRosImuSensor()
{
}

void GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an imu sensor. Exiting.");
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<hrim_sensor_imu_msgs::msg::IMU>("~/out");

  // Create message to be reused
  auto msg = std::make_shared<hrim_sensor_imu_msgs::msg::IMU>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  // TODO(anyone): covariance for IMU's orientation once this is added to gazebo
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->msg_ = msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosImuSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosImuSensorPrivate::OnUpdate()
{
  auto current_time = sensor_->LastUpdateTime();
  auto current_orientation = sensor_->Orientation();
  auto current_velocity = sensor_->AngularVelocity();
  auto current_acceleration = sensor_->LinearAcceleration();

  // Fill message with latest sensor data
  msg_->header.stamp.sec = int(current_time.sec);
  msg_->header.stamp.nanosec = uint(current_time.nsec);

  msg_->orientation.x = current_orientation.X();
  msg_->orientation.y = current_orientation.Y();
  msg_->orientation.z = current_orientation.Z();
  msg_->orientation.w = current_orientation.W();

  msg_->angular_velocity.x = current_velocity.X();
  msg_->angular_velocity.y = current_velocity.Y();
  msg_->angular_velocity.z = current_velocity.Z();

  msg_->linear_acceleration.x = current_acceleration.X();
  msg_->linear_acceleration.y = current_acceleration.Y();
  msg_->linear_acceleration.z = current_acceleration.Z();

  // Publish message
  pub_->publish(msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImuSensor)

}  // namespace gazebo_hrim_plugins
