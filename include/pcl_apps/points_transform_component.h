#ifndef PCL_APPS_POINTS_TRANSFORM_COMPONENT_H_INCLUDED
#define PCL_APPS_POINTS_TRANSFORM_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_POINTS_TRANSFORM_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_POINTS_TRANSFORM_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_POINTS_TRANSFORM_EXPORT __declspec(dllexport)
    #define PCL_APPS_POINTS_TRANSFORM_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_POINTS_TRANSFORM_BUILDING_DLL
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC PCL_APPS_POINTS_TRANSFORM_EXPORT
  #else
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC PCL_APPS_POINTS_TRANSFORM_IMPORT
  #endif
  #define PCL_APPS_POINTS_TRANSFORM_PUBLIC_TYPE PCL_APPS_POINTS_TRANSFORM_PUBLIC
  #define PCL_APPS_POINTS_TRANSFORM_LOCAL
#else
  #define PCL_APPS_POINTS_TRANSFORM_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_POINTS_TRANSFORM_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_POINTS_TRANSFORM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_POINTS_TRANSFORM_PUBLIC
    #define PCL_APPS_POINTS_TRANSFORM_LOCAL
  #endif
  #define PCL_APPS_POINTS_TRANSFORM_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>

namespace pcl_apps
{
    class PointsTransformComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_POINTS_TRANSFORM_PUBLIC
        explicit PointsTransformComponent(const rclcpp::NodeOptions & options);
    private:
        void input(const sensor_msgs::msg::PointCloud2::SharedPtr &in);
        std::string output_frame_id_;
        rclcpp::Clock ros_clock_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
    };
}

#endif  //PCL_APPS_POINTS_TRANSFORM_COMPONENT_H_INCLUDED