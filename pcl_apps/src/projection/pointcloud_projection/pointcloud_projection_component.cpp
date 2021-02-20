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

#include <pcl_apps/projection/pointcloud_projection/pointcloud_projection_component.hpp>

namespace pcl_apps
{
PointcloudProjectionComponent::PointcloudProjectionComponent(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: Node(name, options)
{
  std::string camera_info_topic;
  declare_parameter("camera_info_topic", "/camera_info");
  get_parameter("camera_info_topic", camera_info_topic);
  std::string pointcloud_array_topic;
  declare_parameter("pointcloud_array_topic", "/pointcloud_array");
  get_parameter("pointcloud_array_topic", pointcloud_array_topic);
  sync_ =
    std::shared_ptr<CameraInfoAndPoints>(
    new CameraInfoAndPoints(
      this,
      {camera_info_topic, pointcloud_array_topic},
      std::chrono::milliseconds{100},
      std::chrono::milliseconds{100}));
}

void PointcloudProjectionComponent::callback(
  CameraInfoCallbackT camera_info,
  PointCloudsCallbackT point_clouds)
{
}
}  // namespace pcl_apps
