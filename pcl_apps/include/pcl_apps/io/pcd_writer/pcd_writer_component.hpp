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

#ifndef PCL_APPS__IO__PCD_WRITER__PCD_WRITER_COMPONENT_HPP_
#define PCL_APPS__IO__PCD_WRITER__PCD_WRITER_COMPONENT_HPP_

// Headers in ROS2
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <pcl_apps_msgs/srv/write_pcd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class PcdWriterComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit PcdWriterComponent(const rclcpp::NodeOptions & options);

private:
  PointCloudSubscriber sub_;
  std::string input_topic_;
  bool pointcloud_recieved_;
  bool save_every_pointcloud_;
  rclcpp::Service<pcl_apps_msgs::srv::WritePcd>::SharedPtr server_;
  PCLPointCloudTypePtr cloud_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__IO__PCD_WRITER__PCD_WRITER_COMPONENT_HPP_
