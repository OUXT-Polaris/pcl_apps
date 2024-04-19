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

#ifndef PCL_APPS__FILTER__CROP_HULL_FILTER__CROP_HULL_FILTER_COMPONENT_HPP_
#define PCL_APPS__FILTER__CROP_HULL_FILTER__CROP_HULL_FILTER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_CROP_HULL_FILTER_EXPORT __attribute__((dllexport))
#define PCL_APPS_CROP_HULL_FILTER_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_CROP_HULL_FILTER_EXPORT __declspec(dllexport)
#define PCL_APPS_CROP_HULL_FILTER_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_CROP_HULL_FILTER_BUILDING_DLL
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC PCL_APPS_CROP_HULL_FILTER_EXPORT
#else
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC PCL_APPS_CROP_HULL_FILTER_IMPORT
#endif
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC_TYPE PCL_APPS_CROP_HULL_FILTER_PUBLIC
#define PCL_APPS_CROP_HULL_FILTER_LOCAL
#else
#define PCL_APPS_CROP_HULL_FILTER_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_CROP_HULL_FILTER_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_CROP_HULL_FILTER_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC
#define PCL_APPS_CROP_HULL_FILTER_LOCAL
#endif
#define PCL_APPS_CROP_HULL_FILTER_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/filters/crop_hull.h>

#include <memory>
#include <pcl_apps/adapter.hpp>
#include <pcl_apps_msgs/msg/point_cloud_array.hpp>
#include <pcl_apps_msgs/msg/polygon_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pcl_apps
{
typedef message_filters::Subscriber<pcl_apps_msgs::msg::PolygonArray> PolygonSubscriber;
typedef message_filters::Subscriber<sensor_msgs::msg::PointCloud2> ROS2PointCloudSubscriber;
class CropHullFilterComponent : public rclcpp::Node
{
public:
  PCL_APPS_CROP_HULL_FILTER_PUBLIC
  explicit CropHullFilterComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<pcl_apps_msgs::msg::PointCloudArray>::SharedPtr pointcloud_pub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::PointCloud2, pcl_apps_msgs::msg::PolygonArray>>
    sync_;
  void callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud,
    const pcl_apps_msgs::msg::PolygonArray::ConstSharedPtr polygon);
  std::shared_ptr<PolygonSubscriber> polygon_sub_;
  std::shared_ptr<ROS2PointCloudSubscriber> pointcloud_sub_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__CROP_HULL_FILTER__CROP_HULL_FILTER_COMPONENT_HPP_
