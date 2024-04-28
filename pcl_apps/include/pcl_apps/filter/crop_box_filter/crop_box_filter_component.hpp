// Copyright (c) 2020 OUXT Polaris
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

#ifndef PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_
#define PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_CROP_BOX_FILTER_EXPORT __attribute__((dllexport))
#define PCL_APPS_CROP_BOX_FILTER_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_CROP_BOX_FILTER_EXPORT __declspec(dllexport)
#define PCL_APPS_CROP_BOX_FILTER_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_CROP_BOX_FILTER_BUILDING_DLL
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC PCL_APPS_CROP_BOX_FILTER_EXPORT
#else
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC PCL_APPS_CROP_BOX_FILTER_IMPORT
#endif
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC_TYPE PCL_APPS_CROP_BOX_FILTER_PUBLIC
#define PCL_APPS_CROP_BOX_FILTER_LOCAL
#else
#define PCL_APPS_CROP_BOX_FILTER_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_CROP_BOX_FILTER_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_CROP_BOX_FILTER_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC
#define PCL_APPS_CROP_BOX_FILTER_LOCAL
#endif
#define PCL_APPS_CROP_BOX_FILTER_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <memory>
#include <pcl_apps/adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pcl_apps
{
class CropBoxFilterComponent : public rclcpp::Node
{
public:
  PCL_APPS_CROP_BOX_FILTER_PUBLIC
  explicit CropBoxFilterComponent(const rclcpp::NodeOptions & options);

private:
  void pointsCallback(const PCLPointCloudTypePtr & msg);
  double max_x_, max_y_, max_z_;
  double min_x_, min_y_, min_z_;
  bool keep_organized_, negative_;
  PointCloudPublisher pub_;
  PointCloudSubscriber sub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_
