// Copyright (c) 2019 OUXT Polaris
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

// Headers in this package
#include <pcl_apps/filter/pointcloud_to_laserscan/pointcloud_to_laserscan_component.hpp>
// Headers in RCLCPP
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace pcl_apps
{
PointCloudToLaserScanComponent::PointCloudToLaserScanComponent(const rclcpp::NodeOptions & options)
: Node("pointcloud_to_laserscan", options)
{
  pub_ = create_publisher<sensor_msgs::msg::LaserScan>("output", 1);
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", 1, std::bind(&PointCloudToLaserScanComponent::callback, this, std::placeholders::_1));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 10.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", false);
}

void PointCloudToLaserScanComponent::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header = msg->header;
  scan_msg.angle_min = angle_min_;
  scan_msg.angle_max = angle_max_;
  scan_msg.angle_increment = angle_increment_;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = scan_time_;
  scan_msg.range_min = range_min_;
  scan_msg.range_max = range_max_;
  uint32_t ranges_size =
    std::ceil((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
  if (use_inf_) {
    scan_msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg.ranges.assign(ranges_size, scan_msg.range_max + inf_epsilon_);
  }
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"),
       iter_z(*msg, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_,
        max_height_);
      continue;
    }
    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg.angle_min || angle > scan_msg.angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle,
        scan_msg.angle_min, scan_msg.angle_max);
      continue;
    }
    int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;
    if (range < scan_msg.ranges[index]) {
      scan_msg.ranges[index] = range;
    }
  }
  pub_->publish(scan_msg);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointCloudToLaserScanComponent)
