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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo_hrim_plugins/gazebo_hrim_vacuum_gripper.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hrim_actuator_gripper_msgs/msg/state_gripper.hpp>
#include <hrim_actuator_gripper_srvs/srv/control_vacuum.hpp>

#include <memory>
#include <string>

namespace gazebo_hrim_plugins
{
class GazeboRosVacuumGripperPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Gripper control service callback
  void OnCmdGrip(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlVacuum::Request> request,
    const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlVacuum::Response> response);

  /// Custom implementation of gazebo::physics::GetNearestEntityBelow
  void GetBelow(double &_distBelow);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// StateGripper publisher
  rclcpp::Publisher<hrim_actuator_gripper_msgs::msg::StateGripper>::SharedPtr pub_;

  /// StateGripper message to be published
  std::shared_ptr<hrim_actuator_gripper_msgs::msg::StateGripper> msg_;

  /// ControlVacuum gripper control service
  rclcpp::Service<hrim_actuator_gripper_srvs::srv::ControlVacuum>::SharedPtr cmd_gripper;

  /// Model of the gripper link
  gazebo::physics::ModelPtr model_;

  /// Link of the gripper
  gazebo::physics::LinkPtr gripper_;

  /// Pointer to current world
  gazebo::physics::WorldPtr world_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  /// Current vacuum value, percentage
  float current_vacuum_;

  /// Gripper vacuum force
  double vacuum_force_;

  /// Max gripper vacuum force after normalization
  double max_vacuum_force_;

  /// A mutex to lock access to fields that are used in HRIM message callbacks
  std::mutex lock_;

};

GazeboRosVacuumGripper::GazeboRosVacuumGripper()
: impl_(std::make_unique<GazeboRosVacuumGripperPrivate>())
{
}

GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
}

void GazeboRosVacuumGripper::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Gripper robot model pointer
  impl_->model_ = model;

  // Current world pointer
  impl_->world_ = model->GetWorld();

  // Initialize current_vacuum_
  impl_->current_vacuum_ = 0.0f;

  // Check for a gripper vacuum force in the sdf
  if (sdf->HasElement("vacuum_force")) {
    sdf::ElementPtr force_elem = sdf->GetElement("vacuum_force");
    impl_->vacuum_force_ = force_elem->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Got vacuum force to apply of %s from plugin",std::to_string(impl_->vacuum_force_).c_str());
  }else{
    impl_->vacuum_force_ = 10.0;
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Using default vacuum force of %s",std::to_string(impl_->vacuum_force_).c_str());
  }

  // Check for a max gripper vacuum force in the sdf
  if (sdf->HasElement("max_vacuum_force")) {
    sdf::ElementPtr force_elem = sdf->GetElement("max_vacuum_force");
    impl_->max_vacuum_force_ = force_elem->Get<double>();
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Got vacuum force to apply of %s from plugin",std::to_string(impl_->max_vacuum_force_).c_str());
  }else{
    impl_->max_vacuum_force_ = 50.0;
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Using default vacuum force of %s",std::to_string(impl_->max_vacuum_force_).c_str());
  }

  // Check for gripper link in the sdf
  if (!sdf->HasElement("gripper_link")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Plugin missing <gripper_link>");
    impl_->ros_node_.reset();
    return;
  }

  // Check the model for that link
  sdf::ElementPtr gripper_elem = sdf->GetElement("gripper_link");
  auto gripper_name = gripper_elem->Get<std::string>();
  impl_->gripper_ = model->GetLink(gripper_name);
  if (!impl_->gripper_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "No gripper link found with name [%s].",gripper_name.c_str());
    impl_->ros_node_.reset();
    return;
  }

  // Gripper control service
  impl_->cmd_gripper = impl_->ros_node_->create_service<hrim_actuator_gripper_srvs::srv::ControlVacuum>(
    "~/cmd", std::bind(&GazeboRosVacuumGripperPrivate::OnCmdGrip, impl_.get(),
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Enabled service at [%s] for gripper control",
  impl_->cmd_gripper->get_service_name());

  // Gripper state publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<hrim_actuator_gripper_msgs::msg::StateGripper>("~/state");
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing gripper state at topic [%s]",
  impl_->pub_->get_topic_name());

  // Initialize gripper state message
  impl_->msg_ = std::make_shared<hrim_actuator_gripper_msgs::msg::StateGripper>();
  impl_->msg_->header.frame_id = impl_->gripper_->GetName().c_str();

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosVacuumGripperPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

}

void GazeboRosVacuumGripperPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
    gazebo::common::Time current_time = info.simTime;

    msg_->header.stamp.sec = int(current_time.sec);
    msg_->header.stamp.nanosec = uint(current_time.nsec);

    std::lock_guard<std::mutex> scoped_lock(lock_);

    if(current_vacuum_==0.0){
      msg_->on_off = false;
      pub_->publish(msg_);
      return;
    }else{
      ignition::math::Pose3d parent_pose = gripper_->WorldPose();
      gazebo::physics::Model_V models = world_->Models();

      double distBelow = 0.0;
      GetBelow(distBelow);

      for (size_t i = 0; i < models.size(); i++) {
        if (models[i]->GetName() == gripper_->GetName() ||
            models[i]->GetName() == model_->GetName())
        {
          continue;
        }
        gazebo::physics::Link_V links = models[i]->GetLinks();
        for (size_t j = 0; j < links.size(); j++) {
          ignition::math::Pose3d link_pose = links[j]->WorldPose();
          ignition::math::Pose3d diff = parent_pose - link_pose;
          double norm = diff.Pos().Length();
          if (distBelow < 0.025) {
            links[j]->SetLinearVel(gripper_->WorldLinearVel());
            links[j]->SetAngularVel(gripper_->WorldAngularVel());
            double norm_force = (vacuum_force_*(current_vacuum_/100.0)) / norm;
            if (norm < 0.01) {
              // apply friction like force
              // TODO(unknown): should apply friction actually
              link_pose.Set(parent_pose.Pos(), link_pose.Rot());
              links[j]->SetWorldPose(link_pose);
            }
            if (norm_force > max_vacuum_force_) {
              norm_force = max_vacuum_force_;  // max_force
            }
            ignition::math::Vector3d force = norm_force * diff.Pos().Normalize();
            links[j]->AddForce(force);
          }
        }
      }
      msg_->on_off = true;
      pub_->publish(msg_);
    }
}

// Gripper control service handler
void GazeboRosVacuumGripperPrivate::OnCmdGrip(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlVacuum::Request> request,
  const std::shared_ptr<hrim_actuator_gripper_srvs::srv::ControlVacuum::Response> response)
{
  (void)request_header;
  bool res = false;
  if(request->goal_percentage_suction>=0.0 && request->goal_percentage_suction<=100.0){
    res = true;
    current_vacuum_ = request->goal_percentage_suction;
  }
  response->goal_accepted = res;
}

// Custom implementation of gazebo::physics::GetNearestEntityBelow
void GazeboRosVacuumGripperPrivate::GetBelow(double &_distBelow)
{
  std::string _entityName = "";
  world_->Physics()->InitForThread();
  gazebo::physics::RayShapePtr rayShape = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    world_->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));

  ignition::math::Box box = gripper_->CollisionBoundingBox();
  ignition::math::Vector3d start = gripper_->WorldPose().Pos();
  ignition::math::Vector3d end = start;
  start.Z() = box.Min().Z();
  end.Z() -= 105;
  rayShape->SetPoints(start, end);
  rayShape->GetIntersection(_distBelow, _entityName);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper)
}  // namespace gazebo_plugins
