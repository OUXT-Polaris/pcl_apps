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

// Headers in ROS2
#include <message_synchronizer/message_synchronizer.hpp>
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Headers in STL
#include <array>
#include <memory>
#include <string>

namespace pcl_apps
{
class PointsConcatenateComponent : public rclcpp::Node
{
  using PointCloudType = std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>;
  using AdapterType = rclcpp::TypeAdapter<PointCloudType, sensor_msgs::msg::PointCloud2>;
  using CallbackT = const std::optional<PointCloudType>;

  using Sync2T = message_synchronizer::MessageSynchronizer2<AdapterType, AdapterType>;
  using Sync3T = message_synchronizer::MessageSynchronizer3<AdapterType, AdapterType, AdapterType>;
  using Sync4T =
    message_synchronizer::MessageSynchronizer4<AdapterType, AdapterType, AdapterType, AdapterType>;

public:
  PCL_APPS_PUBLIC
  explicit PointsConcatenateComponent(const rclcpp::NodeOptions & options);

private:
  void callback2(CallbackT in0, CallbackT in1);
  void callback3(CallbackT in0, CallbackT in1, CallbackT in2);
  void callback4(CallbackT in0, CallbackT in1, CallbackT in2, CallbackT in3);
  PointCloudPublisher pub_;
  std::shared_ptr<Sync2T> sync2_;
  std::shared_ptr<Sync3T> sync3_;
  std::shared_ptr<Sync4T> sync4_;
  std::array<std::string, 8> input_topics_;
  int num_input_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__POINTS_CONCATENATE__POINTS_CONCATENATE_COMPONENT_HPP_