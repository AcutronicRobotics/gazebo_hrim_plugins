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

#include <gazebo_hrim_plugins/gazebo_hrim_ray_sensor.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <iostream>
#include <string>
#include <memory>

namespace gazebo_hrim_plugins
{

class GazeboRosRaySensorPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::sensors::RaySensorPtr sensor_;
  gazebo::event::ConnectionPtr sensor_update_event_;

  rclcpp::Publisher<hrim_sensor_lidar_msgs::msg::LidarScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder>::SharedPtr range_specs_pub_;
  rclcpp::Publisher<hrim_sensor_rangefinder_msgs::msg::Distance>::SharedPtr range_pub_;

  std::string frame_name_;
  uint8_t range_radiation_type_;

  rclcpp::Time last_measure_time_;
  rclcpp::Time last_update_time_;

  void OnUpdate();
};

GazeboRosRaySensor::GazeboRosRaySensor()
: impl_(std::make_unique<GazeboRosRaySensorPrivate>())
{
}

GazeboRosRaySensor::~GazeboRosRaySensor()
{
}

void GazeboRosRaySensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an ray sensor. Exiting.");
    return;
  }

  impl_->laser_pub_ = impl_->ros_node_->create_publisher<hrim_sensor_lidar_msgs::msg::LidarScan>("~/outlaser");
  impl_->range_specs_pub_ = impl_->ros_node_->create_publisher<hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder>("~/outspecs");
  impl_->range_pub_ = impl_->ros_node_->create_publisher<hrim_sensor_rangefinder_msgs::msg::Distance>("~/outrange");

  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosRaySensorPrivate::OnUpdate, impl_.get()));

  impl_->last_update_time_ = impl_->last_measure_time_ = rclcpp::Time();

  if (!_sdf->HasElement("radiation_type")) {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "missing <radiation_type>, defaulting to infrared");
    impl_->range_radiation_type_ = hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder::INFRARED;
  } else if ("ultrasound" == _sdf->Get<std::string>("radiation_type")) {
    impl_->range_radiation_type_ = hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder::ULTRASOUND;
  } else if ("infrared" == _sdf->Get<std::string>("radiation_type")) {
    impl_->range_radiation_type_ = hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder::INFRARED;
  } else {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Invalid <radiation_type> [%s]. Can be ultrasound or infrared",
      _sdf->Get<std::string>("radiation_type").c_str());
    return;
  }

}

void GazeboRosRaySensorPrivate::OnUpdate()
{

    hrim_sensor_lidar_msgs::msg::LidarScan::SharedPtr laser_msg_ = std::make_shared<hrim_sensor_lidar_msgs::msg::LidarScan>();
    hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder::SharedPtr range_specs_msg_ = std::make_shared<hrim_sensor_rangefinder_msgs::msg::SpecsRangefinder>();
    hrim_sensor_rangefinder_msgs::msg::Distance::SharedPtr range_msg_ = std::make_shared<hrim_sensor_rangefinder_msgs::msg::Distance>();

    rclcpp::Time prev_meas_ = rclcpp::Time(sensor_->LastMeasurementTime().sec, sensor_->LastMeasurementTime().nsec);
    rclcpp::Time prev_upd_ = rclcpp::Time(sensor_->LastUpdateTime().sec, sensor_->LastUpdateTime().nsec);

    auto sensor_time = sensor_->LastUpdateTime();

    range_specs_msg_->header.frame_id = range_msg_->header.frame_id = laser_msg_->header.frame_id = frame_name_;
    range_specs_msg_->header.stamp.sec = range_msg_->header.stamp.sec = laser_msg_->header.stamp.sec = int(sensor_time.sec);
    range_specs_msg_->header.stamp.nanosec = range_msg_->header.stamp.nanosec = laser_msg_->header.stamp.nanosec = int(sensor_time.nsec);

    range_msg_->time_measurements = laser_msg_->time_increment = (prev_meas_ - last_measure_time_).seconds();
    laser_msg_->time_scan = (prev_upd_ - last_update_time_).seconds();

    laser_msg_->angle_start = sensor_->AngleMin().Radian();
    laser_msg_->angle_end = sensor_->AngleMax().Radian();
    laser_msg_->angle_increment = sensor_->AngleResolution();
    range_specs_msg_->min_range = range_msg_->range_min = laser_msg_->range_min = sensor_->RangeMin();
    range_specs_msg_->max_range = range_msg_->range_max = laser_msg_->range_max = sensor_->RangeMax();

    std::vector<double> tmpRanges;
    sensor_->Ranges(tmpRanges);
    double tmpRange = tmpRanges[0];

    uint index = 0;

    for (double range : tmpRanges) {
      laser_msg_->ranges.push_back(range);
      laser_msg_->intensities.push_back(sensor_->Retro(index));
      if (range < tmpRange) {
        tmpRange = range;
      }
      index++;
    }

    range_msg_->distance = tmpRange;

    range_specs_msg_->radiation_type = range_radiation_type_;
    range_specs_msg_->accuracy = sensor_->RangeResolution();

    auto horizontal_fov = sensor_->AngleMax() - sensor_->AngleMin();
    auto vertical_fov = sensor_->VerticalAngleMax() - sensor_->VerticalAngleMin();
    range_specs_msg_->field_of_view = std::max(horizontal_fov, vertical_fov).Radian();

    laser_pub_->publish(laser_msg_);
    range_specs_pub_->publish(range_specs_msg_);
    range_pub_->publish(range_msg_);

    last_measure_time_ = rclcpp::Time(sensor_->LastMeasurementTime().sec, sensor_->LastMeasurementTime().nsec);
    last_update_time_ = rclcpp::Time(sensor_time.sec, sensor_time.nsec);

}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)

}  // namespace gazebo_hrim_plugins
