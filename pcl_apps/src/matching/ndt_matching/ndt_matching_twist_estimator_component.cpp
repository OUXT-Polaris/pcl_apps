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

#include <pcl_apps/matching/ndt_matching/ndt_matching_twist_estimator_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <string>
#include <vector>
#include <memory>

namespace pcl_apps
{
NdtMatchingTwistEstimatorComponent::NdtMatchingTwistEstimatorComponent(
  const rclcpp::NodeOptions & options)
: Node("ndt_matching_twist_estimator", options)
{
  /* Static Parameters */
  declare_parameter("input_cloud_topic", get_name() + std::string("/input"));
  get_parameter("input_cloud_topic", input_cloud_topic_);
  declare_parameter("input_frame_id", "base_link");
  get_parameter("input_frame_id", input_cloud_frame_id_);
  /* Dynamic Parameters */
  declare_parameter("transform_epsilon", 1.0);
  get_parameter("transform_epsilon", transform_epsilon_);
  declare_parameter("step_size", 0.1);
  get_parameter("step_size", step_size_);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("max_iterations", 35);
  get_parameter("max_iterations", max_iterations_);
  set_on_parameters_set_callback(
    [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult
    {
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
      for (auto param : params) {
        if (param.get_name() == "transform_epsilon") {
          if (transform_epsilon_ > 0) {
            transform_epsilon_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "transform epsilon must over 0";
          }
        }
        if (param.get_name() == "step_size") {
          if (step_size_ > 0) {
            step_size_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "step size must over 0";
          }
        }
        if (param.get_name() == "resolution") {
          if (resolution_ > 0) {
            resolution_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "resolution must over 0";
          }
        }
        if (param.get_name() == "max_iterations") {
          if (max_iterations_ >= 1) {
            max_iterations_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "resolution must over 1";
          }
        }
      }
      if (!results->successful) {
        results->successful = false;
        results->reason = "";
      }
      return *results;
    }
  );
  // Setup Publisher
  std::string output_topic_name = get_name() + std::string("/current_relative_pose");
  current_twist_pub_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_name, 10);
  // Setup Subscriber
  buffer_ = boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::Ptr>(2);
  timestamps_ = boost::circular_buffer<rclcpp::Time>(2);
  auto callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      assert(input_cloud_frame_id_ == msg->header.frame_id);
      timestamps_.push_back(msg->header.stamp);
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
      pcl::fromROSMsg(*msg, *input_cloud);
      buffer_.push_back(input_cloud);
      boost::optional<geometry_msgs::msg::TwistStamped> twist = estimateCurrentTwist();
      if (twist) {
        current_twist_pub_->publish(twist.get());
      }
    };
  sub_input_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10,
      callback);
}

boost::optional<geometry_msgs::msg::TwistStamped> NdtMatchingTwistEstimatorComponent::
estimateCurrentTwist()
{
  assert(timestamps_.size() == buffer_.size());
  if (buffer_.size() == 2) {
    ndt_.setTransformationEpsilon(transform_epsilon_);
    ndt_.setStepSize(step_size_);
    ndt_.setResolution(resolution_);
    ndt_.setMaximumIterations(max_iterations_);
    ndt_.setInputSource(buffer_[1]);
    ndt_.setInputTarget(buffer_[0]);
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 0.0;
    transform.translation.y = 0.0;
    transform.translation.z = 0.0;
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0;
    Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_.align(*output_cloud, mat);
    Eigen::Matrix4f final_transform = ndt_.getFinalTransformation();
    tf2::Matrix3x3 rotation_mat;
    rotation_mat.setValue(static_cast<double>(final_transform(0, 0)),
      static_cast<double>(final_transform(0, 1)),
      static_cast<double>(final_transform(0, 2)), static_cast<double>(final_transform(1, 0)),
      static_cast<double>(final_transform(1, 1)), static_cast<double>(final_transform(1, 2)),
      static_cast<double>(final_transform(2, 0)), static_cast<double>(final_transform(2, 1)),
      static_cast<double>(final_transform(2, 2)));
    tf2::Quaternion quat;
    rotation_mat.getRotation(quat);
    double diff_time = toSec(timestamps_[1]) - toSec(timestamps_[0]);
    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = input_cloud_frame_id_;
    twist.header.stamp = timestamps_[1];
    twist.twist.linear.x = static_cast<double>(final_transform(0, 3)) / diff_time;
    twist.twist.linear.y = static_cast<double>(final_transform(1, 3)) / diff_time;
    twist.twist.linear.z = static_cast<double>(final_transform(2, 3)) / diff_time;
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    twist.twist.angular.x = roll / diff_time;
    twist.twist.angular.y = pitch / diff_time;
    twist.twist.angular.z = yaw / diff_time;
    return twist;
  }
  return boost::none;
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingTwistEstimatorComponent)
