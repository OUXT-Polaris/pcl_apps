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

#ifndef PCL_APPS__IO__PCD_LOADER__PCD_LOADER_COMPONENT_HPP_
#define PCL_APPS__IO__PCD_LOADER__PCD_LOADER_COMPONENT_HPP_

// Headers in ROS2
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Headers in STL
#include <memory>

namespace pcl_apps
{
class PcdLoaderComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit PcdLoaderComponent(const rclcpp::NodeOptions & options);

private:
  PointCloudPublisher pub_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__IO__PCD_LOADER__PCD_LOADER_COMPONENT_HPP_
