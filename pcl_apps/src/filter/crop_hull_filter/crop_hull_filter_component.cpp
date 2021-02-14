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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_apps/filter/crop_hull_filter/crop_hull_filter_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <memory>
#include <vector>

namespace pcl_apps
{
CropHullFilterComponent::CropHullFilterComponent(const rclcpp::NodeOptions & options)
: Node("crop_hull_filter_node", options)
{
  pointcloud_pub_ =
    this->create_publisher<pcl_apps_msgs::msg::PointCloudArray>("points_array", 1);
  polygon_sub_ = std::shared_ptr<PolygonSubscriber>(new PolygonSubscriber(this, "polygon"));
  pointcloud_sub_ =
    std::shared_ptr<PointCloudSubscriber>(new PointCloudSubscriber(this, "points"));
  sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::PointCloud2, pcl_apps_msgs::msg::PolygonArray>>(
    *pointcloud_sub_, *polygon_sub_, 10);
  sync_->registerCallback(
    std::bind(
      &CropHullFilterComponent::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void CropHullFilterComponent::callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud,
  const pcl_apps_msgs::msg::PolygonArray::ConstSharedPtr polygon)
{
  pcl_apps_msgs::msg::PointCloudArray point_clouds;
  point_clouds.header = point_cloud->header;
  pcl::CropHull<pcl::PointXYZI> filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*point_cloud, *pcl_cloud);
  for (auto poly_itr = polygon->polygon.begin(); poly_itr != polygon->polygon.end(); poly_itr++) {
    pcl::ConvexHull<pcl::PointXYZI> convex_hull;
    std::vector<pcl::Vertices> convex_hull_polygons;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> hull_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> hull_points(
      new pcl::PointCloud<pcl::PointXYZI>());
    for (auto point_itr = poly_itr->points.begin(); point_itr != poly_itr->points.end();
      point_itr++)
    {
      pcl::PointXYZI p;
      p.x = point_itr->x;
      p.y = point_itr->y;
      p.z = point_itr->z;
      hull_cloud->push_back(p);
    }
    convex_hull.setInputCloud(hull_cloud);
    convex_hull.reconstruct(*hull_points, convex_hull_polygons);
    filter.setHullIndices(convex_hull_polygons);
    filter.setHullCloud(hull_points);
    filter.setDim(2);
    filter.setCropOutside(false);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filter.setInputCloud(pcl_cloud);
    filter.filter(*filtered);
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered, filtered_msg);
    point_clouds.cloud.push_back(filtered_msg);
  }
  pointcloud_pub_->publish(point_clouds);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::CropHullFilterComponent)
