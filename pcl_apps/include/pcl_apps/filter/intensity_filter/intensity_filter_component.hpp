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

#ifndef PCL_APPS__FILTER__INTENSITY_FILTER__INTENSITY_FILTER_COMPONENT_HPP_
#define PCL_APPS__FILTER__INTENSITY_FILTER__INTENSITY_FILTER_COMPONENT_HPP_

#include <pcl/filters/passthrough.h>

#include <memory>
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pcl_apps
{
class IntensityFilterComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit IntensityFilterComponent(const rclcpp::NodeOptions & options);
  ~IntensityFilterComponent(){};
  void pointsCallback(const PCLPointCloudTypePtr & msg);

private:
  PointCloudPublisher pub_;
  PointCloudSubscriber sub_;
  float min_intensity_;
  float max_intensity_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__FILTER__INTENSITY_FILTER__INTENSITY_FILTER_COMPONENT_HPP_
