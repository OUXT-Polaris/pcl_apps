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

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

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
  std::string output_topic_name;
  declare_parameter("output_topic", get_name() + std::string("/output"));
  get_parameter("output_topic", output_topic_name);
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 10);
  auto callback = [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
      if (msg->header.frame_id == output_frame_id_) {
        pub_->publish(*msg);
      } else {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec));
        geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
          output_frame_id_, msg->header.frame_id, time_point, tf2::durationFromSec(1.0));
        sensor_msgs::msg::PointCloud2 output_msg;
        sensor_msgs::msg::PointCloud2 input_msg = *msg;
        tf2::doTransform(input_msg, output_msg, transform_stamped);
        pub_->publish(output_msg);
      }
    };
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsTransformComponent)
