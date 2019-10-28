// Headers in this package
#include <pcl_apps/filter/radius_outlier_removal/radius_outlier_removal_component.h>

namespace pcl_apps
{
    RadiusOutlierRemovalComponent::RadiusOutlierRemovalComponent(const rclcpp::NodeOptions & options)
    : Node("radius_outlier_removal", options)
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::RadiusOutlierRemovalComponent)