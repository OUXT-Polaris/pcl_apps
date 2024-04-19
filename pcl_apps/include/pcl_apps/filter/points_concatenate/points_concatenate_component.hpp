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

#ifndef PCL_APPS__FILTER__POINTS_CONCATENATE__POINTS_CONCATENATE_COMPONENT_HPP_
#define PCL_APPS__FILTER__POINTS_CONCATENATE__POINTS_CONCATENATE_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_POINTS_CONCATENATE_EXPORT __attribute__((dllexport))
#define PCL_APPS_POINTS_CONCATENATE_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_POINTS_CONCATENATE_EXPORT __declspec(dllexport)
#define PCL_APPS_POINTS_CONCATENATE_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_POINTS_CONCATENATE_BUILDING_DLL
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC PCL_APPS_POINTS_CONCATENATE_EXPORT
#else
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC PCL_APPS_POINTS_CONCATENATE_IMPORT
#endif
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC_TYPE PCL_APPS_POINTS_CONCATENATE_PUBLIC
#define PCL_APPS_POINTS_CONCATENATE_LOCAL
#else
#define PCL_APPS_POINTS_CONCATENATE_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_POINTS_CONCATENATE_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_POINTS_CONCATENATE_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC
#define PCL_APPS_POINTS_CONCATENATE_LOCAL
#endif
#define PCL_APPS_POINTS_CONCATENATE_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS2
#include <message_synchronizer/message_synchronizer.hpp>
#include <pcl_apps/adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Headers in STL
#include <array>
#include <memory>
#include <string>

#include <glog/logging.h>

namespace pcl_apps
{
typedef std::optional<PCLPointCloudType> CallbackT;
typedef message_synchronizer::MessageSynchronizer2<
  PointCloudAdapterType, PointCloudAdapterType, PCLPointCloudType, PCLPointCloudType>
  Sync2T;
typedef message_synchronizer::MessageSynchronizer3<
  PointCloudAdapterType, PointCloudAdapterType, PointCloudAdapterType, PCLPointCloudType,
  PCLPointCloudType, PCLPointCloudType>
  Sync3T;
typedef message_synchronizer::MessageSynchronizer4<
  PointCloudAdapterType, PointCloudAdapterType, PointCloudAdapterType, PointCloudAdapterType,
  PCLPointCloudType, PCLPointCloudType, PCLPointCloudType, PCLPointCloudType>
  Sync4T;

class PointsConcatenateComponent : public rclcpp::Node
{
public:
  PCL_APPS_POINTS_CONCATENATE_PUBLIC
  explicit PointsConcatenateComponent(const rclcpp::NodeOptions & options);

private:
  void callback2(const CallbackT & in0, const CallbackT & in1);
  void callback3(const CallbackT & in0, const CallbackT & in1, const CallbackT & in2);
  void callback4(
    const CallbackT & in0, const CallbackT & in1, const CallbackT & in2, const CallbackT & in3);
  PointCloudPublisher pub_;
  std::shared_ptr<Sync2T> sync2_;
  std::shared_ptr<Sync3T> sync3_;
  std::shared_ptr<Sync4T> sync4_;
  std::array<std::string, 8> input_topics_;
  int num_input_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__POINTS_CONCATENATE__POINTS_CONCATENATE_COMPONENT_HPP_
