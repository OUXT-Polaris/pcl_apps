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

// Headers in ROS2
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <memory>
#include <pcl_apps/projection/pointcloud_projection/pointcloud_projection_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace pcl_apps
{
PointCloudProjectionComponent::PointCloudProjectionComponent(const rclcpp::NodeOptions & options)
: Node("pointcloud_projection_component", options), buffer_(get_clock()), listener_(buffer_)
{
  detection_pub_ = this->create_publisher<perception_msgs::msg::Detection2DArray>("detections", 1);
  std::string camera_info_topic;
  declare_parameter("camera_info_topic", "/camera_info");
  get_parameter("camera_info_topic", camera_info_topic);
  std::string pointcloud_array_topic;
  declare_parameter("pointcloud_array_topic", "/pointcloud_array");
  get_parameter("pointcloud_array_topic", pointcloud_array_topic);
  sync_ = std::shared_ptr<CameraInfoAndPoints>(new CameraInfoAndPoints(
    this, {camera_info_topic, pointcloud_array_topic}, std::chrono::milliseconds{100},
    std::chrono::milliseconds{100}));
  const auto func = std::bind(
    &PointCloudProjectionComponent::callback, this, std::placeholders::_1, std::placeholders::_2);
  sync_->registerCallback(func);
}

PointCloudProjectionComponent::PointCloudProjectionComponent(
  const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  detection_pub_ =
    this->create_publisher<perception_msgs::msg::Detection2DArray>("projected_bbox", 1);
  std::string camera_info_topic;
  declare_parameter("camera_info_topic", "/camera_info");
  get_parameter("camera_info_topic", camera_info_topic);
  std::string pointcloud_array_topic;
  declare_parameter("pointcloud_array_topic", "/pointcloud_array");
  get_parameter("pointcloud_array_topic", pointcloud_array_topic);
  sync_ = std::shared_ptr<CameraInfoAndPoints>(new CameraInfoAndPoints(
    this, {camera_info_topic, pointcloud_array_topic}, std::chrono::milliseconds{100},
    std::chrono::milliseconds{100}));
}

vision_msgs::msg::BoundingBox3D PointCloudProjectionComponent::toBbox(
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud) const
{
  vision_msgs::msg::BoundingBox3D bbox;
  pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
  pcl::PointXYZI min_point;
  pcl::PointXYZI max_point;
  pcl::PointXYZI position;
  Eigen::Matrix3f rotational_matrix;
  feature_extractor.setInputCloud(pointcloud);
  feature_extractor.compute();
  feature_extractor.getAABB(min_point, max_point);
  bbox.center.position.x = (min_point.x + max_point.x) / 2.0;
  bbox.center.position.y = (min_point.y + max_point.y) / 2.0;
  bbox.center.position.z = (min_point.z + max_point.z) / 2.0;
  bbox.center.orientation = geometry_msgs::msg::Quaternion();
  bbox.size.x = std::abs(max_point.x - min_point.x);
  bbox.size.y = std::abs(max_point.y - min_point.y);
  bbox.size.z = std::abs(max_point.z - min_point.z);
  return bbox;
}

void PointCloudProjectionComponent::callback(
  CameraInfoCallbackT camera_info, PointCloudsCallbackT point_clouds)
{
  if (!camera_info || !point_clouds) {
    return;
  }
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info.get());
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_.lookupTransform(
      camera_info.get()->header.frame_id, point_clouds.get()->header.frame_id,
      point_clouds.get()->header.stamp, tf2::durationFromSec(1.0));
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return;
  }
  typedef boost::geometry::model::d2::point_xy<double> point;
  typedef boost::geometry::model::polygon<point> polygon_type;
  typedef boost::geometry::model::box<point> box;
  box camera_bbox(point(0, 0), point(camera_info.get()->width, camera_info.get()->height));
  perception_msgs::msg::Detection2DArray detection_array;
  detection_array.header.frame_id = camera_info.get()->header.frame_id;
  detection_array.header.stamp = = point_clouds.get()->header.stamp;
  for (const auto & point_cloud : point_clouds.get()->cloud) {
    polygon_type poly;
    typedef boost::geometry::ring_type<polygon_type>::type ring_type;
    ring_type & ring = boost::geometry::exterior_ring(poly);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(point_cloud, *cloud);
    for (auto point_itr = cloud->begin(); point_itr != cloud->end(); point_itr++) {
      geometry_msgs::msg::PointStamped p;
      p.point.x = point_itr->x;
      p.point.y = point_itr->y;
      p.point.z = point_itr->z;
      tf2::doTransform(p, p, transform_stamped);
      cv::Point3d point_3d(p.point.x, p.point.y, p.point.z);
      if (point_3d.z > 0) {
        cv::Point2d point_2d = cam_model.project3dToPixel(point_3d);
        ring.push_back(point(point_2d.x, point_2d.y));
      }
    }
    const box bx = boost::geometry::return_envelope<box>(poly);
    box out;
    if (boost::geometry::intersection(camera_bbox, bx, out)) {
      perception_msgs::msg::Detection2D detection;
      detection.header.frame_id = camera_info.get()->header.frame_id;
      detection.header.stamp = point_clouds.get()->header.stamp;
      //      detection.is_tracking = false;
      detection.bbox.center.x = (out.max_corner().x() + out.min_corner().x()) * 0.5;
      detection.bbox.center.y = (out.max_corner().y() + out.min_corner().y()) * 0.5;
      detection.bbox.size_x = out.max_corner().x() - out.min_corner().x();
      detection.bbox.size_y = out.max_corner().y() - out.min_corner().y();
      detection.bbox_3d.emplace_back(toBbox(cloud));
      detection_array.detections.emplace_back(detection);
    }
  }
  detection_pub_->publish(detection_array);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointCloudProjectionComponent)
