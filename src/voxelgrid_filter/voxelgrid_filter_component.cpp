#include <pcl_apps/voxelgrid_filter/voxelgrid_filter_component.h>

namespace pcl_apps
{
    VoxelgridFilterComponent::VoxelgridFilterComponent(const rclcpp::NodeOptions & options)
    : Node("voxelgrid_filter", options)
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::VoxelgridFilterComponent)