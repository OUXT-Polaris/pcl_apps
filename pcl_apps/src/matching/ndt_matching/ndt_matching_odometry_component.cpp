#include <pcl_apps/matching/ndt_matching/ndt_matching_odometry_component.h>

namespace pcl_apps
{
    NdtMatchingOdometryComponent::NdtMatchingOdometryComponent(const rclcpp::NodeOptions & options)
    : Node("ndt_matching_odometry", options)
    {
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingOdometryComponent)