#ifndef PCL_APPS_RADIUS_OUTLIER_REMOVAL_COMPONENT
#define PCL_APPS_RADIUS_OUTLIER_REMOVAL_COMPONENT

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __declspec(dllexport)
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_RADIUS_OUTLIER_REMOVAL_BUILDING_DLL
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT
  #else
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT
  #endif
  #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC_TYPE PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
  #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL
#else
  #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
    #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_LOCAL
  #endif
  #define PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC_TYPE
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
    class RadiusOutlierRemovalComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_RADIUS_OUTLIER_REMOVAL_PUBLIC
        explicit RadiusOutlierRemovalComponent(const rclcpp::NodeOptions & options);
    private:
        
    };
}

#endif  //PCL_APPS_RADIUS_OUTLIER_REMOVAL_COMPONENT