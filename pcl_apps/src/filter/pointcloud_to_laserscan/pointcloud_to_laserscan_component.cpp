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
  pub_ = create_publisher<sensor_msgs::msg::LaserScan>("output", 1);
  sub_ = create_subscription<PointCloudAdapterType>(
    "input", 1, [this](const PCLPointCloudTypePtr & msg) { pointsCallback(msg); });
}

void PointCloudToLaserScanComponent::pointsCallback(const PCLPointCloudTypePtr & msg)
{
  sensor_msgs::msg::LaserScan scan_msg;
  pcl_conversions::fromPCL(msg->header, scan_msg.header);
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
  for (auto itr = msg->begin(); itr != msg->end(); itr++) {
    if (std::isnan(itr->x) || std::isnan(itr->y) || std::isnan(itr->z)) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", itr->x, itr->y, itr->z);
      continue;
    }
    if (itr->z > max_height_ || itr->z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for height %f not in range (%f, %f)\n", itr->z, min_height_,
        max_height_);
      continue;
    }
    double range = hypot(itr->x, itr->y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, itr->x, itr->y, itr->z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(), "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, itr->x, itr->y, itr->z);
      continue;
    }
    double angle = atan2(itr->y, itr->x);
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
