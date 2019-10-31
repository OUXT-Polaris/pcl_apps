#ifndef PCL_APPS_EUCLIDEAN_CLUSTERING_COMPONENT_H_INCLUDED
#define PCL_APPS_EUCLIDEAN_CLUSTERING_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_EXPORT __attribute__ ((dllexport))
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_EXPORT __declspec(dllexport)
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_APPS_EUCLIDEAN_CLUSTERING_BUILDING_DLL
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC PCL_APPS_EUCLIDEAN_CLUSTERING_EXPORT
  #else
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC PCL_APPS_EUCLIDEAN_CLUSTERING_IMPORT
  #endif
  #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC_TYPE PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC
  #define PCL_APPS_EUCLIDEAN_CLUSTERING_LOCAL
#else
  #define PCL_APPS_EUCLIDEAN_CLUSTERING_EXPORT __attribute__ ((visibility("default")))
  #define PCL_APPS_EUCLIDEAN_CLUSTERING_IMPORT
  #if __GNUC__ >= 4
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC
    #define PCL_APPS_EUCLIDEAN_CLUSTERING_LOCAL
  #endif
  #define PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_apps_msgs/msg/point_cloud_array.hpp>

// Headers in PCL
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_apps
{
  class EuclideanClusteringComponent: public rclcpp::Node
  {
  public:
    PCL_APPS_EUCLIDEAN_CLUSTERING_PUBLIC
    explicit EuclideanClusteringComponent(const rclcpp::NodeOptions & options);
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<pcl_apps_msgs::msg::PointCloudArray>::SharedPtr pub_;
    std::string input_topic_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
  };
}

#endif  //PCL_APPS_EUCLIDEAN_CLUSTERING_COMPONENT_H_INCLUDED