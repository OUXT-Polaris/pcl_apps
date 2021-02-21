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

#include <pcl_apps/projection/pointcloud_projection/pointcloud_projection_component.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace pcl_apps
{
PointcloudProjectionComponent::PointcloudProjectionComponent(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  std::string camera_info_topic;
  declare_parameter("camera_info_topic", "/camera_info");
  get_parameter("camera_info_topic", camera_info_topic);
  std::string pointcloud_array_topic;
  declare_parameter("pointcloud_array_topic", "/pointcloud_array");
  get_parameter("pointcloud_array_topic", pointcloud_array_topic);
  sync_ =
    std::shared_ptr<CameraInfoAndPoints>(
    new CameraInfoAndPoints(
      this,
      {camera_info_topic, pointcloud_array_topic},
      std::chrono::milliseconds{100},
      std::chrono::milliseconds{100}));
}

void PointcloudProjectionComponent::callback(
  CameraInfoCallbackT camera_info,
  PointCloudsCallbackT point_clouds)
{
  if (!camera_info || !point_clouds) {
    return;
  }
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info.get());
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_.lookupTransform(
      camera_info->header.frame_id,
      point_clouds->header.frame_id,
      point_clouds->header.stamp,
      tf2::durationFromSec(1.0));
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return;
  }
  typedef boost::geometry::model::d2::point_xy<double> point;
  typedef boost::geometry::model::polygon<point> polygon_type;
  typedef boost::geometry::model::box<point> box;
  box camera_bbox(point(0, 0), point(camera_info->width, camera_info->height));
  vision_msgs::msg::Detection2DArray detection_array;
  for (const auto point_cloud : point_clouds->cloud) {
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
      vision_msgs::msg::Detection2D detection;
      detection.header.frame_id = camera_info->header.frame_id;
      detection.header.stamp = point_clouds->header.stamp;
      detection.is_tracking = false;
      detection.bbox.center.x = (out.max_corner().x() + out.min_corner().x()) * 0.5;
      detection.bbox.center.y = (out.max_corner().y() + out.min_corner().y()) * 0.5;
      detection.bbox.size_x = out.max_corner().x() - out.min_corner().x();
      detection.bbox.size_y = out.max_corner().y() - out.min_corner().y();
      detection_array.detections.emplace_back(detection);
    }
  }
}
}  // namespace pcl_apps
