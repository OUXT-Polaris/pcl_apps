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

#ifndef PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_
#define PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_NDT_MATCHING_EXPORT __attribute__((dllexport))
#define PCL_APPS_NDT_MATCHING_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_NDT_MATCHING_EXPORT __declspec(dllexport)
#define PCL_APPS_NDT_MATCHING_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_NDT_MATCHING_BUILDING_DLL
#define PCL_APPS_NDT_MATCHING_PUBLIC PCL_APPS_NDT_MATCHING_EXPORT
#else
#define PCL_APPS_NDT_MATCHING_PUBLIC PCL_APPS_NDT_MATCHING_IMPORT
#endif
#define PCL_APPS_NDT_MATCHING_PUBLIC_TYPE PCL_APPS_NDT_MATCHING_PUBLIC
#define PCL_APPS_NDT_MATCHING_LOCAL
#else
#define PCL_APPS_NDT_MATCHING_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_NDT_MATCHING_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_NDT_MATCHING_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_NDT_MATCHING_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_NDT_MATCHING_PUBLIC
#define PCL_APPS_NDT_MATCHING_LOCAL
#endif
#define PCL_APPS_NDT_MATCHING_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS2
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/flann_search.h>

#include <boost/shared_ptr.hpp>

// Headers in STL
#include <string>

namespace pcl_apps
{
class NdtMatchingComponent : public rclcpp::Node
{
public:
  PCL_APPS_NDT_MATCHING_PUBLIC
  explicit NdtMatchingComponent(const rclcpp::NodeOptions & options);

private:
  std::string reference_frame_id_;
  std::string reference_cloud_topic_;
  std::string input_cloud_topic_;
  std::string initial_pose_topic_;
  double transform_epsilon_;
  double step_size_;
  double resolution_;
  int max_iterations_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> reference_cloud_;
  bool reference_cloud_recieved_;
  bool initial_pose_recieved_;
  bool use_min_max_filter_;
  std::mutex ndt_map_mtx_;
  double scan_min_range_;
  double scan_max_range_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_reference_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_relative_pose_pub_;
  void updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, rclcpp::Time stamp);
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  geometry_msgs::msg::PoseStamped current_relative_pose_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_
