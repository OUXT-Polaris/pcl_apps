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

#ifndef PCL_APPS__FILTER__POINTS_TRANSFORM__POINTS_TRANSFORM_COMPONENT_HPP_
#define PCL_APPS__FILTER__POINTS_TRANSFORM__POINTS_TRANSFORM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_POINTS_TRANSFORM_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_POINTS_TRANSFORM_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_POINTS_TRANSFORM_EXPORT __declspec(dllexport)
    #define PCL_APPS_POINTS_TRANSFORM_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_POINTS_TRANSFORM_BUILDING_DLL
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC PCL_APPS_POINTS_TRANSFORM_EXPORT
  #else
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC PCL_APPS_POINTS_TRANSFORM_IMPORT
  #endif
  #define PCL_APPS_POINTS_TRANSFORM_PUBLIC_TYPE PCL_APPS_POINTS_TRANSFORM_PUBLIC
  #define PCL_APPS_POINTS_TRANSFORM_LOCAL
#else
  #define PCL_APPS_POINTS_TRANSFORM_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_POINTS_TRANSFORM_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_POINTS_TRANSFORM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC
    #define PCL_APPS_POINTS_TRANSFORM_LOCAL
  #endif
  #define PCL_APPS_POINTS_TRANSFORM_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Headers in PCL
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class PointsTransformComponent : public rclcpp::Node
{
public:
  PCL_APPS_POINTS_TRANSFORM_PUBLIC
  explicit PointsTransformComponent(const rclcpp::NodeOptions & options);

private:
  std::string output_frame_id_;
  rclcpp::Clock ros_clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  void transformPointCloud(
    const Eigen::Matrix4f & transform,
    sensor_msgs::msg::PointCloud2 & in,
    sensor_msgs::msg::PointCloud2 & out);
  std::string input_topic_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__POINTS_TRANSFORM__POINTS_TRANSFORM_COMPONENT_HPP_
