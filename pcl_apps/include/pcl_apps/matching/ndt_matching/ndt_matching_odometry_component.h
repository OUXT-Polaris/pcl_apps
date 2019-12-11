#ifndef PCL_APPS_NDT_MATCHING_ODOMETRY_COMPONENT_H_INCLUDED
#define PCL_APPS_NDT_MATCHING_ODOMETRY_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_EXPORT __declspec(dllexport)
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_NDT_MATCHING_ODOMETRY_BUILDING_DLL
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC PCL_APPS_NDT_MATCHING_ODOMETRY_EXPORT
  #else
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC PCL_APPS_NDT_MATCHING_ODOMETRY_IMPORT
  #endif
  #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC_TYPE PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC
  #define PCL_APPS_NDT_MATCHING_ODOMETRY_LOCAL
#else
  #define PCL_APPS_NDT_MATCHING_ODOMETRY_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_NDT_MATCHING_ODOMETRY_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC
    #define PCL_APPS_NDT_MATCHING_ODOMETRY_LOCAL
  #endif
  #define PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

// Headers in PCL
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace pcl_apps
{
    class NdtMatchingOdometryComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_NDT_MATCHING_ODOMETRY_PUBLIC
        explicit NdtMatchingOdometryComponent(const rclcpp::NodeOptions & options);
    };
}

#endif  //PCL_APPS_NDT_MATCHING_ODOMETRY_COMPONENT_H_INCLUDED