#ifndef PCL_APPS_PCD_WRITER_COMPONENT_H_INCLUDED
#define PCL_APPS_PCD_WRITER_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_PCD_WRITER_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_PCD_WRITER_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_PCD_WRITER_EXPORT __declspec(dllexport)
    #define PCL_APPS_PCD_WRITER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_PCD_WRITER_BUILDING_DLL
    #define PCL_APPS_PCD_WRITER_PUBLIC PCL_APPS_PCD_WRITER_EXPORT
  #else
    #define PCL_APPS_PCD_WRITER_PUBLIC PCL_APPS_PCD_WRITER_IMPORT
  #endif
  #define PCL_APPS_PCD_WRITER_PUBLIC_TYPE PCL_APPS_PCD_WRITER_PUBLIC
  #define PCL_APPS_PCD_WRITER_LOCAL
#else
  #define PCL_APPS_PCD_WRITER_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_PCD_WRITER_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_PCD_WRITER_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_PCD_WRITER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_PCD_WRITER_PUBLIC
    #define PCL_APPS_PCD_WRITER_LOCAL
  #endif
  #define PCL_APPS_PCD_WRITER_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_apps_msgs/srv/write_pcd.hpp>

// Headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pcl_apps
{
  class PcdWriterComponent: public rclcpp::Node
  {
  public:
    PCL_APPS_PCD_WRITER_PUBLIC
    explicit PcdWriterComponent(const rclcpp::NodeOptions & options);
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::string input_topic_;
    pcl::PCLPointCloud2::Ptr cloud_ptr_;
    bool pointcloud_recieved_;
    bool save_every_pointcloud_;
  };
}

#endif  //PCL_APPS_PCD_WRITER_COMPONENT_H_INCLUDED