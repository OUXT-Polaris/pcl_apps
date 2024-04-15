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

#include <pcl_apps/filter/crop_box_filter/crop_box_filter_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in PCL
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_apps
{
CropBoxFilterComponent::CropBoxFilterComponent(const rclcpp::NodeOptions & options)
: Node("crop_box_filter_node", options)
{
  declare_parameter("max_x", 1.0);
  get_parameter("max_x", max_x_);
  declare_parameter("max_y", 1.0);
  get_parameter("max_y", max_y_);
  declare_parameter("max_z", 1.0);
  get_parameter("max_z", max_z_);
  declare_parameter("min_x", -1.0);
  get_parameter("min_x", min_x_);
  declare_parameter("min_y", -1.0);
  get_parameter("min_y", min_y_);
  declare_parameter("min_z", -1.0);
  get_parameter("min_z", min_z_);
  declare_parameter("keep_organized", true);
  get_parameter("keep_organized", keep_organized_);
  declare_parameter("negative", false);
  get_parameter("negative", negative_);
  pub_ = create_publisher<PointCloudAdapterType>("~/points_filtered", 1);
  sub_ = create_subscription<PointCloudAdapterType>(
    "~/points", 1, [this](const PCLPointCloudTypePtr & msg) { pointsCallback(msg); });
}

void CropBoxFilterComponent::pointsCallback(const PCLPointCloudTypePtr & msg)
{
  pcl::CropBox<PCLPointType> filter;
  Eigen::Vector4f new_min_point, new_max_point;
  new_min_point << min_x_, min_y_, min_z_, 0.0;
  new_max_point << max_x_, max_y_, max_z_, 0.0;
  filter.setMin(new_min_point);
  filter.setMax(new_max_point);
  filter.setNegative(negative_);
  filter.setKeepOrganized(keep_organized_);
  filter.setInputCloud(msg);
  PCLPointCloudTypePtr filtered_cloud(new PCLPointCloudType());
  filter.filter(*filtered_cloud);
  pub_->publish(filtered_cloud);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::CropBoxFilterComponent)
