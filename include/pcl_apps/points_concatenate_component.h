#ifndef PCL_APPS_POINTS_CONCATENATE_COMPONENT_H_INCLUDED
#define PCL_APPS_POINTS_CONCATENATE_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_POINTS_CONCATENATE_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_POINTS_CONCATENATE_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_POINTS_CONCATENATE_EXPORT __declspec(dllexport)
    #define PCL_APPS_POINTS_CONCATENATE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_POINTS_CONCATENATE_BUILDING_DLL
    #define PCL_APPS_POINTS_CONCATENATE_PUBLIC PCL_APPS_POINTS_CONCATENATE_EXPORT
  #else
    #define PCL_APPS_POINTS_CONCATENATE_PUBLIC PCL_APPS_POINTS_CONCATENATE_IMPORT
  #endif
  #define PCL_APPS_POINTS_CONCATENATE_PUBLIC_TYPE PCL_APPS_POINTS_CONCATENATE_PUBLIC
  #define PCL_APPS_POINTS_CONCATENATE_LOCAL
#else
  #define PCL_APPS_POINTS_CONCATENATE_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_POINTS_CONCATENATE_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_POINTS_CONCATENATE_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_POINTS_CONCATENATE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_POINTS_CONCATENATE_PUBLIC
    #define PCL_APPS_POINTS_CONCATENATE_LOCAL
  #endif
  #define PCL_APPS_POINTS_CONCATENATE_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in STL
#include <array>

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_apps
{
    typedef sensor_msgs::msg::PointCloud2 PointCloud2;
    typedef message_filters::sync_policies::ApproximateTime
      <PointCloud2, PointCloud2, PointCloud2, PointCloud2, PointCloud2, PointCloud2, PointCloud2, PointCloud2> SyncPolicy;

    class PointsConcatenateComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_POINTS_CONCATENATE_PUBLIC 
            explicit PointsConcatenateComponent(const rclcpp::NodeOptions & options);
    private:
        void input(const PointCloud2::SharedPtr &in0, const PointCloud2::SharedPtr &in1,
          const PointCloud2::SharedPtr &in2, const PointCloud2::SharedPtr &in3,
          const PointCloud2::SharedPtr &in4, const PointCloud2::SharedPtr &in5,
          const PointCloud2::SharedPtr &in6, const PointCloud2::SharedPtr &in7);
        int num_input_;
        std::array<message_filters::Subscriber<PointCloud2>,8> subs_;
        message_filters::PassThrough<PointCloud2> nf_;
    };
}

#endif  //PCL_APPS_POINTS_CONCATENATE_COMPONENT_H_INCLUDED