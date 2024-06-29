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

#include <pcl_apps/filter/intensity_filter/intensity_filter_component.hpp>

namespace pcl_apps
{
IntensityFilterComponent::IntensityFilterComponent(const rclcpp::NodeOptions & options)
: Node("intensity_filter_node", options)
{
  declare_parameter("min_intensity", 0.0);
  get_parameter("min_intensity", min_intensity_);
  declare_parameter("max_intensity", 1.0);
  get_parameter("max_intensity", max_intensity_);
  pub_ = create_publisher<PointCloudAdapterType>("~/points_filtered", 1);
  sub_ = create_subscription<PointCloudAdapterType>(
    "~/points", 1, [this](const PCLPointCloudTypePtr & msg) { pointsCallback(msg); });
}

void IntensityFilterComponent::pointsCallback(const PCLPointCloudTypePtr & msg)
{
  pcl::PassThrough<PCLPointType> pass;
  pass.setInputCloud(msg);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(min_intensity_, max_intensity_);
  PCLPointCloudTypePtr filtered_cloud(new PCLPointCloudType());
  pass.filter(*filtered_cloud);
  pub_->publish(filtered_cloud);
}
}  // namespace pcl_apps

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::IntensityFilterComponent)
