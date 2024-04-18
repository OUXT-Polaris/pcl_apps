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

#ifdef USE_TF2_SENSOR_MSGS_DEPRECATED_HEADER
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

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
  std::string output_topic_name;
  declare_parameter("output_topic", get_name() + std::string("/output"));
  get_parameter("output_topic", output_topic_name);
  pub_ = create_publisher<PointCloudAdapterType>(output_topic_name, 10);
  auto callback = [this](const PCLPointCloudTypePtr msg) -> void {
    std_msgs::msg::Header header;
    pcl_conversions::fromPCL(msg->header, header);
    if (header.frame_id == output_frame_id_) {
      pub_->publish(msg);
    } else {
      try {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(header.stamp.sec) + std::chrono::nanoseconds(header.stamp.nanosec));
        geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
          output_frame_id_, header.frame_id, time_point, tf2::durationFromSec(1.0));
        Eigen::Matrix4f mat =
          tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        pcl::transformPointCloud(*msg, *msg, mat);
        pub_->publish(msg);
      } catch (tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(get_logger(), ex.what());
        return;
      } catch (tf2::LookupException & ex) {
        RCLCPP_ERROR(get_logger(), ex.what());
        return;
      }
    }
  };
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  sub_ = create_subscription<PointCloudAdapterType>(input_topic_, 10, callback);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsTransformComponent)
