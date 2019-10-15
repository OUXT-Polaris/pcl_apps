#include <pcl_apps/points_transform_component.h>

namespace pcl_apps
{
    PointsTransformComponent::PointsTransformComponent(const rclcpp::NodeOptions & options)
    : Node("points_transform", options),
        ros_clock_(RCL_ROS_TIME), 
        buffer_(std::make_shared<rclcpp::Clock>(ros_clock_)),
        listener_(buffer_)
    {

    }

    void PointsTransformComponent::input(const sensor_msgs::msg::PointCloud2::SharedPtr &in)
    {
        pcl::PCLPointCloud2 in_pc;
        pcl_conversions::toPCL(*in, in_pc);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PointsTransformComponent)