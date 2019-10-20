#ifndef PCL_APPS_NDT_MATCHING_COMPONENT_H_INCLUDED
#define PCL_APPS_NDT_MATCHING_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_NDT_MATCHING_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_NDT_MATCHING_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_NDT_MATCHING_EXPORT __declspec(dllexport)
    #define PCL_APPS_NDT_MATCHING_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_NDT_MATCHING_BUILDING_DLL
    #define PCL_APPS_NDT_MATCHING_PUBLIC PCL_APPS_NDT_MATCHING_EXPORT
  #else
    #define PCL_APPS_NDT_MATCHING_PUBLIC PCL_APPS_NDT_MATCHING_IMPORT
  #endif
  #define PCL_APPS_NDT_MATCHING_PUBLIC_TYPE PCL_APPS_NDT_MATCHING_PUBLIC
  #define PCL_APPS_NDT_MATCHING_LOCAL
#else
  #define PCL_APPS_NDT_MATCHING_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_NDT_MATCHING_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_NDT_MATCHING_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_NDT_MATCHING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_NDT_MATCHING_PUBLIC
    #define PCL_APPS_NDT_MATCHING_LOCAL
  #endif
  #define PCL_APPS_NDT_MATCHING_PUBLIC_TYPE
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
#include <tf2_eigen/tf2_eigen.h>

// Headers in PCL
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace pcl_apps
{
    class NdtMatchingComponent: public rclcpp::Node
    {
    public:
        PCL_APPS_NDT_MATCHING_PUBLIC
        explicit NdtMatchingComponent(const rclcpp::NodeOptions & options);
    private:
        std::string reference_frame_id_;
        std::string reference_cloud_topic_;
        std::string input_cloud_topic_;
        double transform_epsilon_;
        double step_size_;
        double resolution_;
        int max_iterations_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;
        bool reference_cloud_recieved_;
        bool initial_pose_recieved_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_reference_cloud_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_cloud_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
        void updateRelativePose(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
        geometry_msgs::msg::PoseStamped current_relative_pose_;
    };
}

#endif  //PCL_APPS_NDT_MATCHING_COMPONENT_H_INCLUDED