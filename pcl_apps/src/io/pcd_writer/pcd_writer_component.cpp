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

#include <pcl_apps/io/pcd_writer/pcd_writer_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <memory>
#include <string>
#include <vector>

namespace pcl_apps
{
PcdWriterComponent::PcdWriterComponent(const rclcpp::NodeOptions & options)
: Node("pcd_writer", options)
{
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  pointcloud_recieved_ = false;
  auto callback = [this](const PCLPointCloudTypePtr msg) -> void {
    cloud_ = msg;
    pointcloud_recieved_ = true;
  };
  sub_ = create_subscription<PointCloudAdapterType>(input_topic_, 10, callback);
  auto write_pcd_callback =
    [this](
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<pcl_apps_msgs::srv::WritePcd::Request> request,
      const std::shared_ptr<pcl_apps_msgs::srv::WritePcd::Response> response) -> void {
    (void)request_header;
    if (pointcloud_recieved_) {
      int result = 0;
      if (request->format == request->ASCII) {
        result = pcl::io::savePCDFileASCII(request->path, *cloud_);
      } else if (request->format == request->BINARY) {
        result = pcl::io::savePCDFileBinary(request->path, *cloud_);
      } else if (request->format == request->BINARY_COMPRESSED) {
        result = pcl::io::savePCDFileBinaryCompressed(request->path, *cloud_);
      } else {
        response->result = response->FAIL;
        response->description = "invalid file format";
      }

      if (result == 0) {
        response->result = response->SUCCESS;
        response->path = request->path;
        response->description = "Succeed to write PCD";
      } else {
        response->result = response->FAIL;
        response->description = "Failed to write PCD";
      }
    } else {
      response->result = response->FAIL;
      response->description = "point cloud does not recieved";
    }
  };
  std::string service_name = get_name() + std::string("/write_pcd");
  server_ = create_service<pcl_apps_msgs::srv::WritePcd>(service_name, write_pcd_callback);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PcdWriterComponent)
