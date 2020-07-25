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

// Headers in this package
#include <pcl_apps/filter/radius_outlier_removal/radius_outlier_removal_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <string>
#include <vector>
#include <memory>

namespace pcl_apps
{
RadiusOutlierRemovalComponent::RadiusOutlierRemovalComponent(const rclcpp::NodeOptions & options)
: Node("radius_outlier_removal", options)
{
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  declare_parameter("search_radius", 1.0);
  get_parameter("search_radius", search_radius_);
  declare_parameter("min_neighbors_in_search_radius", 1);
  get_parameter("min_neighbors_in_search_radius", min_neighbors_in_search_radius_);
  set_on_parameters_set_callback(
    [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult
    {
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
      for (auto param : params) {
        if (param.get_name() == "search_radius") {
          if (search_radius_ > 0) {
            search_radius_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "search radius must over 0";
          }
        }
        if (param.get_name() == "min_neighbors_in_search_radius") {
          if (min_neighbors_in_search_radius_ >= 1) {
            min_neighbors_in_search_radius_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "min neighbors in search radius must over 1";
          }
        }
      }
      if (!results->successful) {
        results->successful = false;
        results->reason = "";
      }
      return *results;
    }
  );
  std::string output_topic_name = get_name() + std::string("/output");
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 10);
  auto callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg, *cloud);
      filter_.setInputCloud(cloud);
      filter_.setRadiusSearch(search_radius_);
      filter_.setMinNeighborsInRadius(min_neighbors_in_search_radius_);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
      filter_.filter(*cloud_filtered);
      pcl::toROSMsg(*cloud_filtered, *msg);
      pub_->publish(*msg);
    };
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::RadiusOutlierRemovalComponent)
