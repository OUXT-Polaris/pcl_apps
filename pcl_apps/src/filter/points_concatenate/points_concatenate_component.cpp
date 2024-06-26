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
  pub_ = create_publisher<PointCloudAdapterType>(output_topic_name, 10);
  for (int i = 0; i < num_input_; i++) {
    declare_parameter(
      "input_topic" + std::to_string(i), get_name() + std::string("/input") + std::to_string(i));
    get_parameter("input_topic" + std::to_string(i), input_topics_[i]);
  }
  const auto get_timestamp = [](const auto & data) {
    return pcl_conversions::fromPCL(data->header).stamp;
  };
  if (num_input_ == 2) {
    sync2_ = std::shared_ptr<Sync2T>(new Sync2T(
      this, {input_topics_[0], input_topics_[1]}, std::chrono::milliseconds{100},
      std::chrono::milliseconds{100},
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(), get_timestamp,
      get_timestamp));
    auto func2 = std::bind(
      &PointsConcatenateComponent::callback2, this, std::placeholders::_1, std::placeholders::_2);
    sync2_->registerCallback(func2);
  } else if (num_input_ == 3) {
    sync3_ = std::shared_ptr<Sync3T>(new Sync3T(
      this, {input_topics_[0], input_topics_[1], input_topics_[2]}, std::chrono::milliseconds{100},
      std::chrono::milliseconds{100},
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(), get_timestamp,
      get_timestamp, get_timestamp));
    auto func3 = std::bind(
      &PointsConcatenateComponent::callback3, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3);
    sync3_->registerCallback(func3);
  } else if (num_input_ == 4) {
    sync4_ = std::shared_ptr<Sync4T>(new Sync4T(
      this, {input_topics_[0], input_topics_[1], input_topics_[2], input_topics_[3]},
      std::chrono::milliseconds{100}, std::chrono::milliseconds{100},
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(), get_timestamp,
      get_timestamp, get_timestamp, get_timestamp));
    auto func4 = std::bind(
      &PointsConcatenateComponent::callback4, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4);
    sync4_->registerCallback(func4);
  }
}

void PointsConcatenateComponent::callback2(CallbackT in0, CallbackT in1)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  bool empty = true;
  if (in0) {
    empty = false;
    *cloud = *in0.value();
  }
  if (in1) {
    empty = false;
    *cloud = *in1.value() + *cloud;
  }
  if (!empty) {
    cloud->header.stamp = pcl_conversions::toPCL(sync2_->getPollTimestamp());
    pub_->publish(cloud);
  }
}

void PointsConcatenateComponent::callback3(CallbackT in0, CallbackT in1, CallbackT in2)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  bool empty = true;
  if (in0) {
    empty = false;
    *cloud = *in0.value();
  }
  if (in1) {
    empty = false;
    *cloud = *in1.value() + *cloud;
  }
  if (in2) {
    empty = false;
    *cloud = *in2.value() + *cloud;
  }
  if (!empty) {
    cloud->header.stamp = pcl_conversions::toPCL(sync3_->getPollTimestamp());
    pub_->publish(cloud);
  }
}

void PointsConcatenateComponent::callback4(
  CallbackT in0, CallbackT in1, CallbackT in2, CallbackT in3)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  bool empty = true;
  if (in0) {
    empty = false;
    *cloud = *in0.value();
  }
  if (in1) {
    empty = false;
    *cloud = *in1.value() + *cloud;
  }
  if (in2) {
    empty = false;
    *cloud = *in2.value() + *cloud;
  }
  if (in3) {
    empty = false;
    *cloud = *in3.value() + *cloud;
  }
  if (!empty) {
    cloud->header.stamp = pcl_conversions::toPCL(sync4_->getPollTimestamp());
    pub_->publish(cloud);
  }
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsConcatenateComponent)
