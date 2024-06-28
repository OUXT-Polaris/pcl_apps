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

#ifndef PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_
#define PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_

#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class RadiusOutlierRemovalComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit RadiusOutlierRemovalComponent(const rclcpp::NodeOptions & options);

private:
  std::string input_topic_;
  PointCloudSubscriber sub_;
  PointCloudPublisher pub_;
  pcl::RadiusOutlierRemoval<PCLPointType> filter_;
  double search_radius_;
  int min_neighbors_in_search_radius_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__RADIUS_OUTLIER_REMOVAL__RADIUS_OUTLIER_REMOVAL_COMPONENT_HPP_
