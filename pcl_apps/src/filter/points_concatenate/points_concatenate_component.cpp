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
  switch (num_input_) {
    case 2:
      sync2_ =
        std::shared_ptr<Sync2T>(
        new Sync2T(
          this, {input_topics_[0], input_topics_[1]},
          std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
      auto func = std::bind(
        &PointsConcatenateComponent::callback2, this,
        std::placeholders::_1,
        std::placeholders::_2);
      sync2_->registerCallback(func);
      break;
    case 3:
      sync3_ =
        std::shared_ptr<Sync3T>(
        new Sync3T(
          this, {input_topics_[0], input_topics_[1], input_topics_[2]},
          std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
      auto func = std::bind(
        &PointsConcatenateComponent::callback3, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3);
      sync3_->registerCallback(func);
      break;
    case 4:
      sync4_ =
        std::shared_ptr<Sync4T>(
        new Sync4T(
          this, {input_topics_[0], input_topics_[1], input_topics_[2], input_topics_[3]},
          std::chrono::milliseconds{100}, std::chrono::milliseconds{30}));
      auto func = std::bind(
        &PointsConcatenateComponent::callback4, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4);
      sync4_->registerCallback(func);
      break;
  }

  /*
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(10));
  for (int i = 0; i < num_input_; i++) {
    declare_parameter(
      "input_topic" + std::to_string(i), get_name() + std::string("/input") + std::to_string(i));
    get_parameter("input_topic" + std::to_string(i), input_topics_[i]);
    boost::shared_ptr<PointCloudSubsciber> sub_ptr =
      boost::make_shared<PointCloudSubsciber>(this, input_topics_[i]);
    sub_ptrs_[i] = sub_ptr;
  }
  switch (num_input_) {
    case 2:
      sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], nf_, nf_, nf_, nf_, nf_, nf_);
      break;
    case 3:
      sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], nf_, nf_, nf_, nf_, nf_);
      break;
    case 4:
      sync_->connectInput(
        *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], nf_, nf_, nf_, nf_);
      break;
    case 5:
      sync_->connectInput(
        *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], nf_, nf_, nf_);
      break;
    case 6:
      sync_->connectInput(
        *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5],
        nf_, nf_);
      break;
    case 7:
      sync_->connectInput(
        *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5],
        *sub_ptrs_[6], nf_);
      break;
    case 8:
      sync_->connectInput(
        *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5],
        *sub_ptrs_[6], *sub_ptrs_[7]);
      break;
  }
  */
}

void PointsConcatenateComponent::callback2(
  CallbackT in0, CallbackT in1)
{

}

void PointsConcatenateComponent::callback3(
  CallbackT in0, CallbackT in1, CallbackT in2)
{

}

void PointsConcatenateComponent::callback4(
  CallbackT in0, CallbackT in1, CallbackT in2,
  CallbackT in3)
{

}

/*
void PointsConcatenateComponent::input(
  const PointCloud2::SharedPtr & in0, const PointCloud2::SharedPtr & in1,
  const PointCloud2::SharedPtr & in2, const PointCloud2::SharedPtr & in3,
  const PointCloud2::SharedPtr & in4, const PointCloud2::SharedPtr & in5,
  const PointCloud2::SharedPtr & in6, const PointCloud2::SharedPtr & in7)
{
  assert(num_input_ >= 2 && num_input_ <= 8);
  pcl::PCLPointCloud2 output_cloud;
  switch (num_input_) {
    case 2: {
        assert(in0->header.frame_id == in1->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        break;
      }
    case 3: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        break;
      }
    case 4: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        assert(in2->header.frame_id == in3->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::PCLPointCloud2 pc3;
        pcl_conversions::toPCL(*in3, pc3);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        pcl::concatenateFields(output_cloud, pc3, output_cloud);
        break;
      }
    case 5: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        assert(in2->header.frame_id == in3->header.frame_id);
        assert(in3->header.frame_id == in4->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::PCLPointCloud2 pc3;
        pcl_conversions::toPCL(*in3, pc3);
        pcl::PCLPointCloud2 pc4;
        pcl_conversions::toPCL(*in4, pc4);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        pcl::concatenateFields(output_cloud, pc3, output_cloud);
        pcl::concatenateFields(output_cloud, pc4, output_cloud);
        break;
      }
    case 6: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        assert(in2->header.frame_id == in3->header.frame_id);
        assert(in3->header.frame_id == in4->header.frame_id);
        assert(in4->header.frame_id == in5->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::PCLPointCloud2 pc3;
        pcl_conversions::toPCL(*in3, pc3);
        pcl::PCLPointCloud2 pc4;
        pcl_conversions::toPCL(*in4, pc4);
        pcl::PCLPointCloud2 pc5;
        pcl_conversions::toPCL(*in5, pc5);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        pcl::concatenateFields(output_cloud, pc3, output_cloud);
        pcl::concatenateFields(output_cloud, pc4, output_cloud);
        pcl::concatenateFields(output_cloud, pc5, output_cloud);
        break;
      }
    case 7: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        assert(in2->header.frame_id == in3->header.frame_id);
        assert(in3->header.frame_id == in4->header.frame_id);
        assert(in4->header.frame_id == in5->header.frame_id);
        assert(in5->header.frame_id == in6->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::PCLPointCloud2 pc3;
        pcl_conversions::toPCL(*in3, pc3);
        pcl::PCLPointCloud2 pc4;
        pcl_conversions::toPCL(*in4, pc4);
        pcl::PCLPointCloud2 pc5;
        pcl_conversions::toPCL(*in5, pc5);
        pcl::PCLPointCloud2 pc6;
        pcl_conversions::toPCL(*in6, pc6);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        pcl::concatenateFields(output_cloud, pc3, output_cloud);
        pcl::concatenateFields(output_cloud, pc4, output_cloud);
        pcl::concatenateFields(output_cloud, pc5, output_cloud);
        pcl::concatenateFields(output_cloud, pc6, output_cloud);
        break;
      }
    case 8: {
        assert(in0->header.frame_id == in1->header.frame_id);
        assert(in1->header.frame_id == in2->header.frame_id);
        assert(in2->header.frame_id == in3->header.frame_id);
        assert(in3->header.frame_id == in4->header.frame_id);
        assert(in4->header.frame_id == in5->header.frame_id);
        assert(in5->header.frame_id == in6->header.frame_id);
        assert(in6->header.frame_id == in7->header.frame_id);
        pcl::PCLPointCloud2 pc0;
        pcl_conversions::toPCL(*in0, pc0);
        pcl::PCLPointCloud2 pc1;
        pcl_conversions::toPCL(*in1, pc1);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*in2, pc2);
        pcl::PCLPointCloud2 pc3;
        pcl_conversions::toPCL(*in3, pc3);
        pcl::PCLPointCloud2 pc4;
        pcl_conversions::toPCL(*in4, pc4);
        pcl::PCLPointCloud2 pc5;
        pcl_conversions::toPCL(*in5, pc5);
        pcl::PCLPointCloud2 pc6;
        pcl_conversions::toPCL(*in6, pc6);
        pcl::PCLPointCloud2 pc7;
        pcl_conversions::toPCL(*in7, pc7);
        pcl::concatenateFields(pc0, pc1, output_cloud);
        pcl::concatenateFields(output_cloud, pc2, output_cloud);
        pcl::concatenateFields(output_cloud, pc3, output_cloud);
        pcl::concatenateFields(output_cloud, pc4, output_cloud);
        pcl::concatenateFields(output_cloud, pc5, output_cloud);
        pcl::concatenateFields(output_cloud, pc6, output_cloud);
        pcl::concatenateFields(output_cloud, pc7, output_cloud);
        break;
      }
  }
  sensor_msgs::msg::PointCloud2 output_cloud_msg;
  pcl_conversions::fromPCL(output_cloud, output_cloud_msg);
  pub_->publish(output_cloud_msg);
}
*/
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsConcatenateComponent)
