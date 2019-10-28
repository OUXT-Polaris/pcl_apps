#include <pcl_apps/clustering/euclidean_clustering/euclidean_clustering_component.h>

namespace pcl_apps
{
    EuclideanClusteringComponent::EuclideanClusteringComponent(const rclcpp::NodeOptions & options)
    : Node("euclidean_clustering", options)
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::EuclideanClusteringComponent)