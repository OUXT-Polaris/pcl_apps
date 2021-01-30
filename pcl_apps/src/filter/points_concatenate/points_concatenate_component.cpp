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

#include <pcl_apps/filter/points_concatenate/points_concatenate_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <memory>
#include <string>
namespace pcl_apps
{
PointsConcatenateComponent::PointsConcatenateComponent(const rclcpp::NodeOptions & options)
: Node("points_concatenate", options)
{
  declare_parameter("num_input", 2);
  get_parameter("num_input", num_input_);
  assert(num_input_ >= 2 && num_input_ <= 4);
  std::string output_topic_name = get_name() + std::string("/output");
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 10);
  for (int i = 0; i < num_input_; i++) {
    declare_parameter(
      "input_topic" + std::to_string(i), get_name() + std::string("/input") + std::to_string(i));
    get_parameter("input_topic" + std::to_string(i), input_topics_[i]);
  }
  if (num_input_ == 2) {
    sync2_ =
      std::shared_ptr<Sync2T>(
      new Sync2T(
        this, {input_topics_[0], input_topics_[1]},
        std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
    auto func2 = std::bind(
      &PointsConcatenateComponent::callback2, this,
      std::placeholders::_1,
      std::placeholders::_2);
    sync2_->registerCallback(func2);
  } else if (num_input_ == 3) {
    sync3_ =
      std::shared_ptr<Sync3T>(
      new Sync3T(
        this, {input_topics_[0], input_topics_[1], input_topics_[2]},
        std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
    auto func3 = std::bind(
      &PointsConcatenateComponent::callback3, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3);
    sync3_->registerCallback(func3);
  } else if (num_input_ == 4) {
    sync4_ =
      std::shared_ptr<Sync4T>(
      new Sync4T(
        this, {input_topics_[0], input_topics_[1], input_topics_[2], input_topics_[3]},
        std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
    auto func4 = std::bind(
      &PointsConcatenateComponent::callback4, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4);
    sync4_->registerCallback(func4);
  }
}

void PointsConcatenateComponent::callback2(
  CallbackT in0, CallbackT in1)
{
  pcl::PCLPointCloud2 cloud;
  if (in0) {
    const PointCloud2Ptr pc = in0.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in1) {
    const PointCloud2Ptr pc = in1.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  sensor_msgs::msg::PointCloud2 output_cloud_msg;
  pcl_conversions::fromPCL(cloud, output_cloud_msg);
  pub_->publish(output_cloud_msg);
}

void PointsConcatenateComponent::callback3(
  CallbackT in0, CallbackT in1, CallbackT in2)
{
  pcl::PCLPointCloud2 cloud;
  if (in0) {
    const PointCloud2Ptr pc = in0.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in1) {
    const PointCloud2Ptr pc = in1.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in2) {
    const PointCloud2Ptr pc = in2.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  sensor_msgs::msg::PointCloud2 output_cloud_msg;
  pcl_conversions::fromPCL(cloud, output_cloud_msg);
  pub_->publish(output_cloud_msg);
}

void PointsConcatenateComponent::callback4(
  CallbackT in0, CallbackT in1, CallbackT in2,
  CallbackT in3)
{
  pcl::PCLPointCloud2 cloud;
  if (in0) {
    const PointCloud2Ptr pc = in0.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in1) {
    const PointCloud2Ptr pc = in1.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in2) {
    const PointCloud2Ptr pc = in2.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  if (in3) {
    const PointCloud2Ptr pc = in3.get();
    pcl::PCLPointCloud2 pc_cloud;
    pcl_conversions::toPCL(*pc, pc_cloud);
    pcl::concatenateFields(pc_cloud, cloud, cloud);
  }
  sensor_msgs::msg::PointCloud2 output_cloud_msg;
  pcl_conversions::fromPCL(cloud, output_cloud_msg);
  pub_->publish(output_cloud_msg);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsConcatenateComponent)
