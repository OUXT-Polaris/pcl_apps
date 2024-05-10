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

#ifndef PCL_APPS__FILTER__POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_COMPONENT_HPP_
#define PCL_APPS__FILTER__POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_COMPONENT_HPP_

#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_apps
{
class PointCloudToLaserScanComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit PointCloudToLaserScanComponent(const rclcpp::NodeOptions & options);

private:
  void pointsCallback(const PCLPointCloudTypePtr & msg);
  PointCloudSubscriber sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  double inf_epsilon_;
  bool use_inf_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_COMPONENT_HPP_
