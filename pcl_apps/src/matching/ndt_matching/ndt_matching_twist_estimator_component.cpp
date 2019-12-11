#include <pcl_apps/matching/ndt_matching/ndt_matching_twist_estimator_component.h>

namespace pcl_apps
{
    NdtMatchingTwistEstimatorComponent::NdtMatchingTwistEstimatorComponent(const rclcpp::NodeOptions & options)
    : Node("ndt_matching_twist_estimator", options)
    {
        /* Static Parameters */
        declare_parameter("input_cloud_topic",get_name() + std::string("/input"));
        get_parameter("input_cloud_topic",input_cloud_topic_);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::NdtMatchingTwistEstimatorComponent)