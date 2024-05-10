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

#ifndef PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_
#define PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_

// Headers in ROS2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

#include <pcl_apps/adapter.hpp>
#ifdef USE_TF2_EIGEN_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif
#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#ifdef USE_TF2_SENSOR_MSGS_DEPRECATED_HEADER
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <pcl_apps/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/flann_search.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>

// Headers in STL
#include <string>

namespace pcl_apps
{
class NdtMatchingComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit NdtMatchingComponent(const rclcpp::NodeOptions & options);

private:
  std::string reference_frame_id_;
  std::string base_frame_id_;
  std::string reference_cloud_topic_;
  std::string input_cloud_topic_;
  std::string initial_pose_topic_;
  double transform_epsilon_;
  double step_size_;
  double resolution_;
  int max_iterations_;
  PCLPointCloudTypePtr reference_cloud_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  bool reference_cloud_recieved_;
  bool initial_pose_recieved_;
  bool use_min_max_filter_;
  std::mutex ndt_map_mtx_;
  int omp_num_thread_;
  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose);
  PointCloudSubscriber sub_reference_cloud_;
  PointCloudSubscriber sub_input_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_relative_pose_pub_;
  void updateRelativePose(PCLPointCloudTypePtr input_cloud, rclcpp::Time stamp);
  std::shared_ptr<pclomp::NormalDistributionsTransform<PCLPointType, PCLPointType>> ndt_;
  geometry_msgs::msg::PoseStamped current_relative_pose_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_COMPONENT_HPP_
