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

// Headers in ROS2
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>

#ifdef USE_TF2_EIGEN_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class PointsTransformComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit PointsTransformComponent(const rclcpp::NodeOptions & options);

private:
  std::string output_frame_id_;
  rclcpp::Clock ros_clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  PointCloudSubscriber sub_;
  PointCloudPublisher pub_;
  std::string input_topic_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__POINTS_TRANSFORM__POINTS_TRANSFORM_COMPONENT_HPP_
