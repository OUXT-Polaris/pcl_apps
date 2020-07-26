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

#include <pcl_apps/filter/points_transform/points_transform_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <limits>
#include <string>

namespace pcl_apps
{
PointsTransformComponent::PointsTransformComponent(const rclcpp::NodeOptions & options)
: Node("points_transform", options),
  ros_clock_(RCL_ROS_TIME),
  buffer_(get_clock()),
  listener_(buffer_)
{
  declare_parameter("output_frame_id", "");
  get_parameter("output_frame_id", output_frame_id_);
  std::string output_topic_name = get_name() + std::string("/output");
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 10);
  auto callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
    if (msg->header.frame_id == output_frame_id_) {
      pub_->publish(*msg);
    } else {
      tf2::TimePoint time_point = tf2::TimePoint(
        std::chrono::seconds(msg->header.stamp.sec) +
        std::chrono::nanoseconds(msg->header.stamp.nanosec));
      geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
        msg->header.frame_id, output_frame_id_, time_point, tf2::durationFromSec(1.0));
      Eigen::Matrix4f mat =
        tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
      pcl::PointCloud<pcl::PointXYZI> pc_out;
      sensor_msgs::msg::PointCloud2 output_msg;
      transformPointCloud(mat, *msg, output_msg);
      pub_->publish(output_msg);
    }
  };
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
}

void PointsTransformComponent::transformPointCloud(
  const Eigen::Matrix4f & transform, sensor_msgs::msg::PointCloud2 & in,
  sensor_msgs::msg::PointCloud2 & out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex(in, "x");
  int y_idx = pcl::getFieldIndex(in, "y");
  int z_idx = pcl::getFieldIndex(in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    RCLCPP_ERROR(
      get_logger(), "Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (
    in.fields[x_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    in.fields[y_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    in.fields[z_idx].datatype != sensor_msgs::msg::PointField::FLOAT32) {
    RCLCPP_ERROR(
      get_logger(), "X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex(in, "distance");

  // Copy the other data
  if (&in != &out) {
    out.header = in.header;
    out.height = in.height;
    out.width = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step = in.point_step;
    out.row_step = in.row_step;
    out.is_dense = in.is_dense;
    out.data.resize(in.data.size());
    // Copy everything as it's faster than copying individual elements
    memcpy(&out.data[0], &in.data[0], in.data.size());
  }

  Eigen::Array4i xyz_offset(
    in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i) {
    Eigen::Vector4f pt(
      *reinterpret_cast<float *>(&in.data[xyz_offset[0]]),
      *reinterpret_cast<float *>(&in.data[xyz_offset[1]]),
      *reinterpret_cast<float *>(&in.data[xyz_offset[2]]), 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = i * in.point_step + in.fields[dist_idx].offset;
    float * distance_ptr =
      (dist_idx < 0 ? NULL : reinterpret_cast<float *>(&in.data[distance_ptr_offset]));
    if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2])) {
      if (distance_ptr == NULL || !std::isfinite(*distance_ptr)) {  // Invalid point
        pt_out = pt;
      } else {                  // max range point
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
      }
    } else {
      pt_out = transform * pt;
    }

    if (max_range_point) {
      // Save x value in distance again
      *reinterpret_cast<float *>(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy(&out.data[xyz_offset[0]], &pt_out[0], sizeof(float));
    memcpy(&out.data[xyz_offset[1]], &pt_out[1], sizeof(float));
    memcpy(&out.data[xyz_offset[2]], &pt_out[2], sizeof(float));

    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex(in, "vp_x");
  if (vp_idx != -1) {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i) {
      float * pstep =
        reinterpret_cast<float *>(&out.data[i * out.point_step + out.fields[vp_idx].offset]);
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in(pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsTransformComponent)
