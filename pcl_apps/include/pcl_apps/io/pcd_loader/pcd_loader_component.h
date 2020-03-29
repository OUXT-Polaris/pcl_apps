#ifndef PCL_APPS_PCD_LOADER_COMPONENT_H_INCLUDED
#define PCL_APPS_PCD_LOADER_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_PCD_LOADER_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_PCD_LOADER_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_PCD_LOADER_EXPORT __declspec(dllexport)
    #define PCL_APPS_PCD_LOADER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_PCD_LOADER_BUILDING_DLL
    #define PCL_APPS_PCD_LOADER_PUBLIC PCL_APPS_PCD_LOADER_EXPORT
  #else
    #define PCL_APPS_PCD_LOADER_PUBLIC PCL_APPS_PCD_LOADER_IMPORT
  #endif
  #define PCL_APPS_PCD_LOADER_PUBLIC_TYPE PCL_APPS_PCD_LOADER_PUBLIC
  #define PCL_APPS_PCD_LOADER_LOCAL
#else
  #define PCL_APPS_PCD_LOADER_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_PCD_LOADER_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_PCD_LOADER_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_PCD_LOADER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_PCD_LOADER_PUBLIC
    #define PCL_APPS_PCD_LOADER_LOCAL
  #endif
  #define PCL_APPS_PCD_LOADER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

// Headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pcl_apps
{
class PcdLoaderComponent : public rclcpp::Node
{
public:
  PCL_APPS_PCD_LOADER_PUBLIC
  explicit PcdLoaderComponent(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
};
}

#endif  //PCL_APPS_PCD_LOADER_COMPONENT_H_INCLUDED
