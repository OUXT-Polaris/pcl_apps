#include <pcl_apps/io/pcd_loader/pcd_loader_component.h>

namespace pcl_apps
{
    PcdLoaderComponent::PcdLoaderComponent(const rclcpp::NodeOptions & options)
    : Node("points_transform", options)
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PcdLoaderComponent)