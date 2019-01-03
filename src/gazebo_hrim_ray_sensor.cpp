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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
  rclcpp::Publisher<hrim_sensor_3dcameratof_msgs::msg::PointCloud>::SharedPtr cloud_pub_;

  std::string frame_name_;
  uint8_t range_radiation_type_;

  rclcpp::Time last_measure_time_;
  rclcpp::Time last_update_time_;

  void OnUpdate();
  void ConvertCloud(const hrim_sensor_3dcameratof_msgs::msg::PointCloud::SharedPtr & pointCloud, double min_intensity);
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
  impl_->cloud_pub_ = impl_->ros_node_->create_publisher<hrim_sensor_3dcameratof_msgs::msg::PointCloud>("~/outcloud");

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
    hrim_sensor_3dcameratof_msgs::msg::PointCloud::SharedPtr cloud_msg_ = std::make_shared<hrim_sensor_3dcameratof_msgs::msg::PointCloud>();

    rclcpp::Time prev_meas_ = rclcpp::Time(sensor_->LastMeasurementTime().sec, sensor_->LastMeasurementTime().nsec);
    rclcpp::Time prev_upd_ = rclcpp::Time(sensor_->LastUpdateTime().sec, sensor_->LastUpdateTime().nsec);

    auto sensor_time = sensor_->LastUpdateTime();

    cloud_msg_->header.frame_id = range_specs_msg_->header.frame_id = range_msg_->header.frame_id = laser_msg_->header.frame_id = frame_name_;
    cloud_msg_->header.stamp.sec = range_specs_msg_->header.stamp.sec = range_msg_->header.stamp.sec = laser_msg_->header.stamp.sec = int(sensor_time.sec);
    cloud_msg_->header.stamp.nanosec = range_specs_msg_->header.stamp.nanosec = range_msg_->header.stamp.nanosec = laser_msg_->header.stamp.nanosec = int(sensor_time.nsec);

    range_msg_->time_measurements = laser_msg_->time_increment = (prev_meas_ - last_measure_time_).seconds();
    laser_msg_->time_scan = (prev_upd_ - last_update_time_).seconds();

    laser_msg_->angle_start = sensor_->AngleMin().Degree();
    laser_msg_->angle_end = sensor_->AngleMax().Degree();
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
    range_specs_msg_->field_of_view = (sensor_->AngleMax() - sensor_->AngleMin()).Degree();
    range_specs_msg_->accuracy = sensor_->RangeResolution();

    GazeboRosRaySensorPrivate::ConvertCloud(cloud_msg_, 0.0);

    laser_pub_->publish(laser_msg_);
    range_specs_pub_->publish(range_specs_msg_);
    range_pub_->publish(range_msg_);
    cloud_pub_->publish(cloud_msg_);

    last_measure_time_ = rclcpp::Time(sensor_->LastMeasurementTime().sec, sensor_->LastMeasurementTime().nsec);
    last_update_time_ = rclcpp::Time(sensor_time.sec, sensor_time.nsec);

}

void GazeboRosRaySensorPrivate::ConvertCloud(const hrim_sensor_3dcameratof_msgs::msg::PointCloud::SharedPtr & pointCloud, double min_intensity = 0.0){  // Create message to send
  sensor_msgs::msg::PointCloud2 pc;

  // Pointcloud will be dense, unordered
  pc.height = 1;
  pc.is_dense = true;

  // Fill header
  // pc.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());

  // Cache values that are repeatedly used

  // auto count = in.scan().count();
  uint count = sensor_->RangeCount();
  // auto vertical_count = in.scan().vertical_count();
  uint vertical_count = sensor_->VerticalRangeCount();
  // auto angle_step = in.scan().angle_step();
  auto angle_step = sensor_->AngleResolution();
  // auto vertical_angle_step = in.scan().vertical_angle_step();
  auto vertical_angle_step = sensor_->VerticalAngleResolution();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step)) {
    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "Infinite angle step results in wrong PointCloud2");
  }

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(pc);
  pcd_modifier.setPointCloud2Fields(4,
    // "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "x", 1, hrim_sensor_3dcameratof_msgs::msg::PointField::FLOAT32,
    // "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, hrim_sensor_3dcameratof_msgs::msg::PointField::FLOAT32,
    // "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, hrim_sensor_3dcameratof_msgs::msg::PointField::FLOAT32,
    // "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    "intensity", 1, hrim_sensor_3dcameratof_msgs::msg::PointField::FLOAT32);
  pcd_modifier.resize(vertical_count * count);
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc, "intensity");

  // Iterators to range and intensities
  std::vector<double> tmpRanges;
  sensor_->Ranges(tmpRanges);

  std::vector<double> tmpIntensities;
  for (uint index = 0; index < count; index++) {
    tmpIntensities.push_back(sensor_->Retro(index));
  }

  // auto range_iter = in.scan().ranges().begin();
  auto range_iter = tmpRanges.begin();
  // auto intensity_iter = in.scan().intensities().begin();
  auto intensity_iter = tmpIntensities.begin();

  // Number of points actually added
  size_t points_added = 0;

  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;

  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  // for (j = 0, inclination = in.scan().vertical_angle_min();
  for (j = 0, inclination = sensor_->VerticalAngleMin().Radian();
    j < vertical_count;
    ++j, inclination += vertical_angle_step)
  {
    double c_inclination = cos(inclination);
    double s_inclination = sin(inclination);
    // for (i = 0, azimuth = in.scan().angle_min();
    for (i = 0, azimuth = sensor_->AngleMin().Radian();
      i < count;
      ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
    {
      double c_azimuth = cos(azimuth);
      double s_azimuth = sin(azimuth);

      double r = *range_iter;
      // Skip NaN / inf points
      if (!std::isfinite(r)) {
        continue;
      }

      // Get intensity, clipping at min_intensity
      double intensity = *intensity_iter;
      if (intensity < min_intensity) {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;

      // Increment ouput iterators
      ++points_added;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  pcd_modifier.resize(points_added);

  pointCloud->height = pc.height;
  pointCloud->width = pc.width;
  pointCloud->is_bigendian = pc.is_bigendian;
  pointCloud->point_step = pc.point_step;
  pointCloud->row_step = pc.row_step;
  pointCloud->is_dense = pc.is_dense;
  pointCloud->data = pc.data;

  std::vector<hrim_sensor_3dcameratof_msgs::msg::PointField> tmpFields;

  for(auto field : pc.fields){
    hrim_sensor_3dcameratof_msgs::msg::PointField tmpField;
    tmpField.name = field.name;
    tmpField.offset = field.offset;
    tmpField.datatype = field.datatype;
    tmpField.count = field.count;
    tmpFields.push_back(tmpField);
  }

  pointCloud->fields = tmpFields;

}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)

}  // namespace gazebo_hrim_plugins
