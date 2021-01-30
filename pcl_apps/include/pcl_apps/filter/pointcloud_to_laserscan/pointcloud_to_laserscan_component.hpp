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

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_EXPORT __attribute__((dllexport))
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_EXPORT __declspec(dllexport)
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_POINT_CLOUD_TO_LASERSCAN_BUILDING_DLL
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC PCL_APPS_POINT_CLOUD_TO_LASERSCAN_EXPORT
#else
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC PCL_APPS_POINT_CLOUD_TO_LASERSCAN_IMPORT
#endif
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC_TYPE PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_LOCAL
#else
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_LOCAL
#endif
#define PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace pcl_apps
{
class PointCloudToLaserScanComponent : public rclcpp::Node
{
public:
  PCL_APPS_POINT_CLOUD_TO_LASERSCAN_PUBLIC
  explicit PointCloudToLaserScanComponent(const rclcpp::NodeOptions & options);

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
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
