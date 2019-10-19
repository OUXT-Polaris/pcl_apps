#include <pcl_apps/voxelgrid_filter/voxelgrid_filter_component.h>

namespace pcl_apps
{
    VoxelgridFilterComponent::VoxelgridFilterComponent(const rclcpp::NodeOptions & options)
    : Node("voxelgrid_filter", options)
    {
        declare_parameter("leaf_size",1.0);
        get_parameter("leaf_size",leaf_size_);
        set_on_parameters_set_callback(
        [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult 
        {
            auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
            bool param_updated = false;
            for(auto param : params)
            {
                if(param.get_name() == "leaf_size")
                {
                    param_updated = true;
                    leaf_size_ = param.as_double();
                    results->successful = true;
                    results->reason = "";
                }
            }
            if(param_updated)
            {
                results->successful = false;
                results->reason = "";
            }
            return *results;
        }
        );
        std::string output_topic_name = get_name() + std::string("/output");
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name,10);
        leaf_size_ = 0.01;
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(*msg,*cloud);
            filter_.setInputCloud(cloud);
            filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
            filter_.filter(*cloud_filtered);
            sensor_msgs::msg::PointCloud2 output_cloud_msg;
            pcl_conversions::fromPCL(*cloud_filtered,output_cloud_msg);
            pub_->publish(output_cloud_msg);
        };
        std::string input_topic_name = get_name() + std::string("/input");
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_name, 10, callback);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::VoxelgridFilterComponent)