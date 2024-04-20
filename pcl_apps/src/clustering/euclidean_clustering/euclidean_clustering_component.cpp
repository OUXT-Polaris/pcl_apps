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

#include <pcl_apps/clustering/euclidean_clustering/euclidean_clustering_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <memory>
#include <string>
#include <vector>

namespace pcl_apps
{
EuclideanClusteringComponent::EuclideanClusteringComponent(const rclcpp::NodeOptions & options)
: Node("euclidean_clustering", options)
{
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  declare_parameter("cluster_tolerance", 1.0);
  get_parameter("cluster_tolerance", cluster_tolerance_);
  declare_parameter("min_cluster_size", 1);
  get_parameter("min_cluster_size", min_cluster_size_);
  declare_parameter("max_cluster_size", 10000);
  get_parameter("max_cluster_size", max_cluster_size_);
  param_handler_ptr_ = add_on_set_parameters_callback(
    [this](
      const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult {
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
      for (auto param : params) {
        if (param.get_name() == "cluster_tolerance") {
          if (cluster_tolerance_ > 0) {
            cluster_tolerance_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "cluster tolerance must be over 0";
          }
        }
        if (param.get_name() == "min_cluster_size") {
          if (min_cluster_size_ >= 1) {
            min_cluster_size_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "min cluster size must be over 0";
          }
        }
        if (param.get_name() == "max_cluster_size") {
          if (max_cluster_size_ >= 1) {
            max_cluster_size_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "max cluster size must be over 0";
          }
        }
      }
      if (!results->successful) {
        results->successful = false;
        results->reason = "";
      }
      return *results;
    });
  std::string output_topic_name = get_name() + std::string("/output");
  pub_ = create_publisher<pcl_apps_msgs::msg::PointCloudArray>(output_topic_name, 10);
  auto callback = [this](const PCLPointCloudTypePtr cloud) -> void {
    pcl_apps_msgs::msg::PointCloudArray clusters;
    clusters.header = pcl_conversions::fromPCL(cloud->header);
    // pcl::PointCloud<PCLPointType>::Ptr cloud;
    // pcl::fromROSMsg(*msg, *cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PCLPointType> clustering;
    pcl::search::KdTree<PCLPointType>::Ptr tree(new pcl::search::KdTree<PCLPointType>);
    tree->setInputCloud(cloud);
    clustering.setClusterTolerance(cluster_tolerance_);
    clustering.setMinClusterSize(min_cluster_size_);
    clustering.setMaxClusterSize(max_cluster_size_);
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(cloud);
    clustering.extract(cluster_indices);
    for (auto cluster_itr = cluster_indices.begin(); cluster_itr != cluster_indices.end();
         cluster_itr++) {
      pcl::PointCloud<PCLPointType> pointcloud;
      pointcloud.width = cluster_itr->indices.size();
      pointcloud.height = 1;
      pointcloud.is_dense = false;
      pointcloud.points.resize(pointcloud.width * pointcloud.height);
      for (size_t i = 0; i < cluster_itr->indices.size(); ++i) {
        double x = cloud->points[cluster_itr->indices[i]].x;
        double y = cloud->points[cluster_itr->indices[i]].y;
        double z = cloud->points[cluster_itr->indices[i]].z;
        double intensity = cloud->points[cluster_itr->indices[i]].intensity;
        PCLPointType p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        pointcloud.points[i] = p;
      }
      sensor_msgs::msg::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(pointcloud, pointcloud_msg);
      pointcloud_msg.header = pcl_conversions::fromPCL(cloud->header);
      clusters.cloud.push_back(pointcloud_msg);
    }
    pub_->publish(clusters);
  };
  sub_ = create_subscription<PointCloudAdapterType>(input_topic_, 10, callback);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::EuclideanClusteringComponent)
