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

#include <pcl_apps/io/pcd_loader/pcd_loader_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <string>

namespace pcl_apps
{
PcdLoaderComponent::PcdLoaderComponent(const rclcpp::NodeOptions & options)
: Node("pcd_loader", options)
{
  std::string file_path;
  declare_parameter("file_path", std::string(""));
  get_parameter("file_path", file_path);
  std::string output_topic;
  declare_parameter("output_topic", get_name() + std::string("/output"));
  get_parameter("output_topic", output_topic);
  std::string inference_id;
  declare_parameter("inference_id", std::string("map"));
  get_parameter("inference_id", inference_id);
  pcl::PCLPointCloud2 cloud;
  int result = pcl::io::loadPCDFile(file_path, cloud);
  if (result == 0) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl_conversions::fromPCL(cloud, msg);
    msg.header.frame_id = inference_id;
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::QoS(10).transient_local());
    pub_->publish(msg);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to load ", file_path);
  }
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PcdLoaderComponent)
