// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
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

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosDiffDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };

  /// Indicates which wheel
  enum
  {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Distance between the wheels, in meters.
  double wheel_separation_;

  /// Diameter of wheels, in meters.
  double wheel_diameter_;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque_;

  /// Maximum wheel acceleration
  double max_wheel_accel_;

  /// Desired wheel speed.
  double desired_wheel_speed_[2];

  /// Speed sent to wheel.
  double wheel_speed_instr_[2];

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_x_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update_;

  /// Either ENCODER or WORLD
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;
};

GazeboRosDiffDrive::GazeboRosDiffDrive()
: impl_(std::make_unique<GazeboRosDiffDrivePrivate>())
{
}

GazeboRosDiffDrive::~GazeboRosDiffDrive()
{
}

void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get joints
  impl_->joints_.resize(2);

  auto left_joint = _sdf->Get<std::string>("left_joint", "left_joint").first;
  impl_->joints_[GazeboRosDiffDrivePrivate::LEFT] = _model->GetJoint(left_joint);

  auto right_joint = _sdf->Get<std::string>("right_joint", "right_joint").first;
  impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT] = _model->GetJoint(right_joint);

  if (!impl_->joints_[GazeboRosDiffDrivePrivate::LEFT] ||
    !impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT])
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Joint [%s] or [%s] not found, plugin will not work.", left_joint, right_joint);

    impl_->ros_node_.reset();
    return;
  }

  // Kinematic properties
  impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation", 0.34).first;
  impl_->wheel_diameter_ = _sdf->Get<double>("wheel_diameter", 0.15).first;

  impl_->desired_wheel_speed_[GazeboRosDiffDrivePrivate::RIGHT] = 0;
  impl_->desired_wheel_speed_[GazeboRosDiffDrivePrivate::LEFT] = 0;
  impl_->wheel_speed_instr_[GazeboRosDiffDrivePrivate::RIGHT] = 0;
  impl_->wheel_speed_instr_[GazeboRosDiffDrivePrivate::LEFT] = 0;

  // Dynamic properties
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

  impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
  impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT]->SetParam("fmax", 0, impl_->max_wheel_torque_);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = 1;
  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(),
    std::placeholders::_1),
    qos_profile);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<GazeboRosDiffDrivePrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos_profile);

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(
      new tf2_ros::TransformBroadcaster(impl_->ros_node_));

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    if (impl_->publish_wheel_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "Publishing wheel transforms between [%s], [%s] and [%s]", impl_->robot_base_frame_.c_str(),
        impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->GetName().c_str(),
        impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT]->GetName().c_str());
    }
  }

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosDiffDrive::Reset()
{
  if (impl_->joints_[GazeboRosDiffDrivePrivate::LEFT] &&
    impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT])
  {
    impl_->last_update_time_ =
      impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->GetWorld()->SimTime();
    impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[GazeboRosDiffDrivePrivate::RIGHT]->SetParam("fmax", 0, impl_->max_wheel_torque_);
  }
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;
}

void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  // Update encoder even if we're going to skip this update
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  // Update odom message if using ground truth
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }

  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }

  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }

  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }

  // Update robot in case new velocities have been requested
  UpdateWheelVelocities();

  // Current speed
  double current_speed[2];
  current_speed[LEFT] = joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
  current_speed[RIGHT] = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);

  // If max_accel == 0, or target speed is reached
  if (max_wheel_accel_ == 0 ||
    (fabs(desired_wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01) ||
    (fabs(desired_wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01))
  {
    joints_[LEFT]->SetParam("vel", 0, desired_wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
    joints_[RIGHT]->SetParam("vel", 0, desired_wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
  } else {
    if (desired_wheel_speed_[LEFT] >= current_speed[LEFT]) {
      wheel_speed_instr_[LEFT] += fmin(desired_wheel_speed_[LEFT] - current_speed[LEFT],
          max_wheel_accel_ * seconds_since_last_update);
    } else {
      wheel_speed_instr_[LEFT] += fmax(desired_wheel_speed_[LEFT] - current_speed[LEFT],
          -max_wheel_accel_ * seconds_since_last_update);
    }

    if (desired_wheel_speed_[RIGHT] > current_speed[RIGHT]) {
      wheel_speed_instr_[RIGHT] += fmin(desired_wheel_speed_[RIGHT] - current_speed[RIGHT],
          max_wheel_accel_ * seconds_since_last_update);
    } else {
      wheel_speed_instr_[RIGHT] += fmax(desired_wheel_speed_[RIGHT] - current_speed[RIGHT],
          -max_wheel_accel_ * seconds_since_last_update);
    }

    joints_[LEFT]->SetParam("vel", 0, wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
    joints_[RIGHT]->SetParam("vel", 0, wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
  }

  last_update_time_ = _info.simTime;
}

void GazeboRosDiffDrivePrivate::UpdateWheelVelocities()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double vr = target_x_;
  double va = target_rot_;

  desired_wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
  desired_wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
}

void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = wheel_separation_;

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = vl * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosDiffDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (auto i : {LEFT, RIGHT}) {
    auto pose_wheel = joints_[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}  // namespace gazebo_plugins
