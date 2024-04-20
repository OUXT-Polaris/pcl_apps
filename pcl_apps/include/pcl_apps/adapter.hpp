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

#include <pcl_type_adapter/pcl_type_adapter.hpp>

namespace pcl_apps
{
using PCLPointType = pcl::PointXYZI;
using PCLPointCloudType = pcl::PointCloud<PCLPointType>;
using PCLPointCloudTypePtr = std::shared_ptr<PCLPointCloudType>;
using PointCloudAdapterType =
  rclcpp::TypeAdapter<PCLPointCloudTypePtr, sensor_msgs::msg::PointCloud2>;
using PointCloudPublisher = std::shared_ptr<rclcpp::Publisher<PointCloudAdapterType>>;
using PointCloudSubscriber = std::shared_ptr<rclcpp::Subscription<PointCloudAdapterType>>;
}  // namespace pcl_apps
