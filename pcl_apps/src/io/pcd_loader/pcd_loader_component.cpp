#include <pcl_apps/io/pcd_loader/pcd_loader_component.h>

namespace pcl_apps
{
    PcdLoaderComponent::PcdLoaderComponent(const rclcpp::NodeOptions & options)
    : Node("pcd_loader", options)
    {
        std::string file_path;
        declare_parameter("file_path","");
        get_parameter("file_path",file_path);
        std::string output_topic;
        declare_parameter("output_topic",get_name() + std::string("/output"));
        pcl::PCLPointCloud2 cloud;
        int result = pcl::io::loadPCDFile(file_path,cloud);
        if(result == 0)
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl_conversions::fromPCL(cloud,msg);
            pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(10).transient_local());
            pub_->publish(msg);
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"Failed to load ",file_path);
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PcdLoaderComponent)