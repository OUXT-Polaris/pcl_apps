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

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PCL_APPS_POINTCLOUD_PROJECTION_EXPORT __attribute__((dllexport))
#define PCL_APPS_POINTCLOUD_PROJECTION_IMPORT __attribute__((dllimport))
#else
#define PCL_APPS_POINTCLOUD_PROJECTION_EXPORT __declspec(dllexport)
#define PCL_APPS_POINTCLOUD_PROJECTION_IMPORT __declspec(dllimport)
#endif
#ifdef PCL_APPS_POINTCLOUD_PROJECTION_BUILDING_DLL
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC PCL_APPS_POINTCLOUD_PROJECTION_EXPORT
#else
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC PCL_APPS_POINTCLOUD_PROJECTION_IMPORT
#endif
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC_TYPE PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC
#define PCL_APPS_POINTCLOUD_PROJECTION_LOCAL
#else
#define PCL_APPS_POINTCLOUD_PROJECTION_EXPORT __attribute__((visibility("default")))
#define PCL_APPS_POINTCLOUD_PROJECTION_IMPORT
#if __GNUC__ >= 4
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC __attribute__((visibility("default")))
#define PCL_APPS_POINTCLOUD_PROJECTION_LOCAL __attribute__((visibility("hidden")))
#else
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC
#define PCL_APPS_POINTCLOUD_PROJECTION_LOCAL
#endif
#define PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <message_synchronizer/message_synchronizer.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl_apps_msgs/msg/point_cloud_array.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

namespace pcl_apps
{
typedef sensor_msgs::msg::CameraInfo CameraInfoT;
typedef std::shared_ptr<CameraInfoT> CameraInfoTPtr;
typedef pcl_apps_msgs::msg::PointCloudArray PointCloudArrayT;
typedef std::shared_ptr<PointCloudArrayT> PointCloudArrayTPtr;

typedef message_synchronizer::MessageSynchronizer2<CameraInfoT, PointCloudArrayT>
  CameraInfoAndPoints;
typedef const boost::optional<const CameraInfoTPtr> & CameraInfoCallbackT;
typedef const boost::optional<const PointCloudArrayTPtr> & PointCloudsCallbackT;

class PointCloudProjectionComponent : public rclcpp::Node
{
public:
  PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC
  explicit PointCloudProjectionComponent(
    const std::string & name, const rclcpp::NodeOptions & options);
  PCL_APPS_POINTCLOUD_PROJECTION_PUBLIC
  explicit PointCloudProjectionComponent(const rclcpp::NodeOptions & options);

private:
  vision_msgs::msg::BoundingBox3D toBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud) const;
  rclcpp::Publisher<perception_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  std::shared_ptr<CameraInfoAndPoints> sync_;
  void callback(CameraInfoCallbackT camera_info, PointCloudsCallbackT point_clouds);
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__PROJECTION__POINTCLOUD_PROJECTION__POINTCLOUD_PROJECTION_COMPONENT_HPP_
