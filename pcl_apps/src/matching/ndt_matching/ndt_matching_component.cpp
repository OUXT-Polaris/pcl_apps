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

#include <pcl_apps/matching/ndt_matching/ndt_matching_component.hpp>

// Headers in ROS2
#include <rclcpp_components/register_node_macro.hpp>

// Headers in STL
#include <memory>
#include <string>
#include <vector>

namespace pcl_apps
{
NdtMatchingComponent::NdtMatchingComponent(const rclcpp::NodeOptions & options)
: Node("ndt_matching", options)
{
  /* Static Parameters */
  declare_parameter("reference_frame_id", "map");
  get_parameter("reference_frame_id", reference_frame_id_);
  declare_parameter("base_frame_id", "base_link");
  get_parameter("base_frame_id", base_frame_id_);
  declare_parameter("reference_cloud_topic", get_name() + std::string("/reference"));
  get_parameter("reference_cloud_topic", reference_cloud_topic_);
  declare_parameter("input_cloud_topic", get_name() + std::string("/input"));
  get_parameter("input_cloud_topic", input_cloud_topic_);
  declare_parameter("initial_pose_topic", get_name() + std::string("/initial_pose"));
  get_parameter("initial_pose_topic", initial_pose_topic_);
  /* Dynamic Parameters */
  declare_parameter("transform_epsilon", 1.0);
  get_parameter("transform_epsilon", transform_epsilon_);
  declare_parameter("step_size", 0.1);
  get_parameter("step_size", step_size_);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("max_iterations", 35);
  get_parameter("max_iterations", max_iterations_);
  declare_parameter("omp_num_thread", 8);
  get_parameter("omp_num_thread", omp_num_thread_);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  ndt_ = std::make_shared<pclomp::NormalDistributionsTransform<PCLPointType, PCLPointType>>();
  ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  if (0 < omp_num_thread_) ndt_->setNumThreads(omp_num_thread_);

  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  param_handler_ptr_ = add_on_set_parameters_callback(
    [this](
      const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult {
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
    });
  /* Setup Publisher */
  std::string output_topic_name = get_name() + std::string("/current_pose");
  current_relative_pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_name, 1);

  /* Setup Subscriber */
  reference_cloud_recieved_ = false;
  initial_pose_recieved_ = false;

  auto reference_cloud_callback = [this](const PCLPointCloudTypePtr reference_cloud_) -> void {
    ndt_map_mtx_.lock();
    initial_pose_recieved_ = false;
    reference_cloud_recieved_ = true;
    ndt_->setInputTarget(reference_cloud_);
    ndt_map_mtx_.unlock();
  };
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
    initial_pose_recieved_ = true;
    current_relative_pose_ = *msg;
  };
  auto callback = [this](const PCLPointCloudTypePtr input_cloud) -> void {
    std::vector<int> nan_index;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, nan_index);
    publishTF(reference_frame_id_, base_frame_id_, current_relative_pose_);
    updateRelativePose(input_cloud, pcl_conversions::fromPCL(input_cloud->header.stamp));
    current_relative_pose_pub_->publish(current_relative_pose_);
  };
  sub_reference_cloud_ = create_subscription<PointCloudAdapterType>(
    reference_cloud_topic_, rclcpp::QoS{1}.transient_local(), reference_cloud_callback);
  sub_input_cloud_ = create_subscription<PointCloudAdapterType>(input_cloud_topic_, 10, callback);
  sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    initial_pose_topic_, 1, initial_pose_callback);
}

void NdtMatchingComponent::updateRelativePose(PCLPointCloudTypePtr input_cloud, rclcpp::Time stamp)
{
  ndt_->setTransformationEpsilon(transform_epsilon_);
  ndt_->setStepSize(step_size_);
  ndt_->setResolution(resolution_);
  ndt_->setMaximumIterations(max_iterations_);
  ndt_->setInputSource(input_cloud);
  geometry_msgs::msg::Transform transform;
  transform.translation.x = current_relative_pose_.pose.position.x;
  transform.translation.y = current_relative_pose_.pose.position.y;
  transform.translation.z = current_relative_pose_.pose.position.z;
  transform.rotation = current_relative_pose_.pose.orientation;
  Eigen::Matrix4f mat = tf2::transformToEigen(transform).matrix().cast<float>();
  auto output_cloud = std::make_shared<pcl::PointCloud<PCLPointType>>();
  ndt_->align(*output_cloud, mat);
  Eigen::Matrix4f final_transform = ndt_->getFinalTransformation();
  tf2::Matrix3x3 rotation_mat;
  rotation_mat.setValue(
    static_cast<double>(final_transform(0, 0)), static_cast<double>(final_transform(0, 1)),
    static_cast<double>(final_transform(0, 2)), static_cast<double>(final_transform(1, 0)),
    static_cast<double>(final_transform(1, 1)), static_cast<double>(final_transform(1, 2)),
    static_cast<double>(final_transform(2, 0)), static_cast<double>(final_transform(2, 1)),
    static_cast<double>(final_transform(2, 2)));
  tf2::Quaternion quat;
  rotation_mat.getRotation(quat);
  current_relative_pose_.header.stamp = stamp;
  current_relative_pose_.header.frame_id = reference_frame_id_;
  current_relative_pose_.pose.position.x = static_cast<double>(final_transform(0, 3));
  current_relative_pose_.pose.position.y = static_cast<double>(final_transform(1, 3));
  current_relative_pose_.pose.position.z = static_cast<double>(final_transform(2, 3));
  current_relative_pose_.pose.orientation.x = quat.x();
  current_relative_pose_.pose.orientation.y = quat.y();
  current_relative_pose_.pose.orientation.z = quat.z();
  current_relative_pose_.pose.orientation.w = quat.w();
}

void NdtMatchingComponent::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = frame_id;
  transform_stamped.header.stamp = pose.header.stamp;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.pose.position.x;
  transform_stamped.transform.translation.y = pose.pose.position.y;
  transform_stamped.transform.translation.z = pose.pose.position.z;
  transform_stamped.transform.rotation.w = pose.pose.orientation.w;
  transform_stamped.transform.rotation.x = pose.pose.orientation.x;
  transform_stamped.transform.rotation.y = pose.pose.orientation.y;
  transform_stamped.transform.rotation.z = pose.pose.orientation.z;
  broadcaster_->sendTransform(transform_stamped);
}
}  // namespace pcl_apps

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingComponent)
