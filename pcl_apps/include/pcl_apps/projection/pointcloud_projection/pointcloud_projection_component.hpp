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

#ifndef PCL_APPS__PROJECTION__POINTCLOUD_PROJECTION__POINTCLOUD_PROJECTION_COMPONENT_HPP_
#define PCL_APPS__PROJECTION__POINTCLOUD_PROJECTION__POINTCLOUD_PROJECTION_COMPONENT_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <message_synchronizer/message_synchronizer.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <pcl_apps_msgs/msg/point_cloud_array.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pcl_apps
{
class PointCloudProjectionComponent : public rclcpp::Node
{
  using CameraInfoT = sensor_msgs::msg::CameraInfo;
  using CameraInfoTPtr = std::shared_ptr<CameraInfoT>;
  using PointCloudArrayT = pcl_apps_msgs::msg::PointCloudArray;
  using PointCloudArrayTPtr = std::shared_ptr<PointCloudArrayT>;

  using CameraInfoAndPoints =
    message_synchronizer::MessageSynchronizer2<CameraInfoT, PointCloudArrayT>;
  using CameraInfoCallbackT = const std::optional<const CameraInfoT> &;
  using PointCloudsCallbackT = const std::optional<const PointCloudArrayT> &;

public:
  PCL_APPS_PUBLIC
  explicit PointCloudProjectionComponent(
    const std::string & name, const rclcpp::NodeOptions & options);
  PCL_APPS_PUBLIC
  explicit PointCloudProjectionComponent(const rclcpp::NodeOptions & options);

private:
  vision_msgs::msg::BoundingBox3D toBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud) const;
  rclcpp::Publisher<perception_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray toMarker(
    const perception_msgs::msg::Detection2DArray & detections) const;
  std::shared_ptr<CameraInfoAndPoints> sync_;
  void callback(CameraInfoCallbackT camera_info, PointCloudsCallbackT point_clouds);
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__PROJECTION__POINTCLOUD_PROJECTION__POINTCLOUD_PROJECTION_COMPONENT_HPP_