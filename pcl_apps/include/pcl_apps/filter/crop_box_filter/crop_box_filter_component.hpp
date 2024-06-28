// Copyright (c) 2020 OUXT Polaris
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

#ifndef PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_
#define PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pcl_apps
{
class CropBoxFilterComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit CropBoxFilterComponent(const rclcpp::NodeOptions & options);

private:
  void pointsCallback(const PCLPointCloudTypePtr & msg);
  double max_x_, max_y_, max_z_;
  double min_x_, min_y_, min_z_;
  bool keep_organized_, negative_;
  PointCloudPublisher pub_;
  PointCloudSubscriber sub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__CROP_BOX_FILTER__CROP_BOX_FILTER_COMPONENT_HPP_
