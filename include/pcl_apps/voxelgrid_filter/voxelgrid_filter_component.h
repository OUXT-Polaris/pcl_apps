#ifndef PCL_APPS_VOEXLGRID_FILTER_COMPONENT_COMPONENT_H_INCLUDED
#define PCL_APPS_VOEXLGRID_FILTER_COMPONENT_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_VOEXLGRID_FILTER_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_VOEXLGRID_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_VOEXLGRID_FILTER_EXPORT __declspec(dllexport)
    #define PCL_APPS_VOEXLGRID_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_VOEXLGRID_FILTER_BUILDING_DLL
    #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC PCL_APPS_VOEXLGRID_FILTER_EXPORT
  #else
    #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC PCL_APPS_VOEXLGRID_FILTER_IMPORT
  #endif
  #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC_TYPE PCL_APPS_VOEXLGRID_FILTER_PUBLIC
  #define PCL_APPS_VOEXLGRID_FILTER_LOCAL
#else
  #define PCL_APPS_VOEXLGRID_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_VOEXLGRID_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_VOEXLGRID_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC
    #define PCL_APPS_VOEXLGRID_FILTER_LOCAL
  #endif
  #define PCL_APPS_VOEXLGRID_FILTER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_apps
{
    class VoxelgridFilterComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_VOEXLGRID_FILTER_PUBLIC
        explicit VoxelgridFilterComponent(const rclcpp::NodeOptions & options);
    };
}

#endif  //PCL_APPS_VOEXLGRID_FILTER_COMPONENT_COMPONENT_H_INCLUDED