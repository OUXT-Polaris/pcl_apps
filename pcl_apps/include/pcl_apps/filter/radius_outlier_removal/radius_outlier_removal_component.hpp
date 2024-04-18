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

#ifndef PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_
#define PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __attribute__((dllexport))
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __declspec(dllexport)
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_RADIUS_OUTLIER_REMOVAL_BUILDING_DLL
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT
#else
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT
#endif
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC_TYPE PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL
#else
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL
#endif
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <pcl_apps/adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class RadiusOutlierRemovalComponent : public rclcpp::Node
{
public:
  PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
  explicit RadiusOutlierRemovalComponent(const rclcpp::NodeOptions & options);

private:
  std::string input_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> filter_;
  double search_radius_;
  int min_neighbors_in_search_radius_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_
