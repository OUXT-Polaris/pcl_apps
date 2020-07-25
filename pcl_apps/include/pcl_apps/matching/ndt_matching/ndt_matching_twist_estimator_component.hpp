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

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_EXPORT __declspec(dllexport)
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_BUILDING_DLL
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC \
  PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_EXPORT
  #else
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC \
  PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_IMPORT
  #endif
  #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC_TYPE \
  PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC
  #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_LOCAL
#else
  #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC
    #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_LOCAL
  #endif
  #define PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

// Headers in PCL
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

// Headers in STL
#include <string>

namespace pcl_apps
{
class NdtMatchingTwistEstimatorComponent : public rclcpp::Node
{
public:
  PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_PUBLIC
  explicit NdtMatchingTwistEstimatorComponent(const rclcpp::NodeOptions & options);

private:
  std::string input_cloud_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_pub_;
  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer_;
  boost::circular_buffer<rclcpp::Time> timestamps_;
  double transform_epsilon_;
  double step_size_;
  double resolution_;
  int max_iterations_;
  std::string input_cloud_frame_id_;
  boost::optional<geometry_msgs::msg::TwistStamped> estimateCurrentTwist();
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
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
};
}  // namespace pcl_apps

#endif  // PCL_APPS__MATCHING__NDT_MATCHING__NDT_MATCHING_TWIST_ESTIMATOR_COMPONENT_HPP_
