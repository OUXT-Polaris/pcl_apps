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

#ifndef PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_TWIST_ESTIMATOR_COMPONENT_HPP_
#define PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_TWIST_ESTIMATOR_COMPONENT_HPP_

// Headers in ROS2
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

#include <pcl_apps/visibility_control.hpp>
#ifdef USE_TF2_EIGEN_DEPRECATED_HEADER
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pcl_apps/adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Headers in PCL
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>

// Headers in STL
#include <string>

namespace pcl_apps
{
class NdtMatchingTwistEstimatorComponent : public rclcpp::Node
{
public:
  PCL_APPS_PUBLIC
  explicit NdtMatchingTwistEstimatorComponent(const rclcpp::NodeOptions & options);

private:
  std::string input_cloud_topic_;
  PointCloudSubscriber sub_input_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_pub_;
  boost::circular_buffer<pcl::PointCloud<PCLPointType>::Ptr> buffer_;
  boost::circular_buffer<rclcpp::Time> timestamps_;
  double transform_epsilon_;
  double step_size_;
  double resolution_;
  int max_iterations_;
  std::string input_cloud_frame_id_;
  std::optional<geometry_msgs::msg::TwistStamped> estimateCurrentTwist();
  pcl::NormalDistributionsTransform<PCLPointType, PCLPointType> ndt_;
  double toSec(rclcpp::Duration duration)
  {
    double ret;
    double nsecs = static_cast<double>(duration.nanoseconds());
    ret = nsecs * std::pow(static_cast<double>(10.0), -9);
    return ret;
  }
  double toSec(rclcpp::Time stamp)
  {
    double ret;
    double nsecs = static_cast<double>(stamp.nanoseconds());
    ret = nsecs * std::pow(static_cast<double>(10.0), -9);
    return ret;
  }
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_ptr_;
};
}  // namespace pcl_apps

#endif  // PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_TWIST_ESTIMATOR_COMPONENT_HPP_
