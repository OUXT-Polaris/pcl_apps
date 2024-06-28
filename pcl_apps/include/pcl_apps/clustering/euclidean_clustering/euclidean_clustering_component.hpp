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

#ifndef PCL_APPS__CLUSTERING__EUCLIDEAN_CLUSTERING__EUCLIDEAN_CLUSTERING_COMPONENT_HPP_
#define PCL_APPS__CLUSTERING__EUCLIDEAN_CLUSTERING__EUCLIDEAN_CLUSTERING_COMPONENT_HPP_

// Headers in ROS2
#include <pcl_apps/adapter.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <pcl_apps_msgs/msg/point_cloud_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class EuclideanClusteringComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit EuclideanClusteringComponent(const rclcpp::NodeOptions & options);

private:
  PointCloudSubscriber sub_;
  rclcpp::Publisher<pcl_apps_msgs::msg::PointCloudArray>::SharedPtr pub_;
  std::string input_topic_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__CLUSTERING__EUCLIDEAN_CLUSTERING__EUCLIDEAN_CLUSTERING_COMPONENT_HPP_
