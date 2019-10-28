#include <pcl_apps/io/pcd_writer/pcd_writer_component.h>

namespace pcl_apps
{
    PcdWriterComponent::PcdWriterComponent(const rclcpp::NodeOptions & options)
    : Node("pcd_writer", options)
    {
        declare_parameter("input_topic",get_name() + std::string("/input"));
        get_parameter("input_topic",input_topic_);
        pointcloud_recieved_ = false;
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::fromROSMsg(*msg, cloud_);
            pointcloud_recieved_ = true;
        };
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
        auto write_pcd_callback = 
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<pcl_apps_msgs::srv::WritePcd::Request> request,
        const std::shared_ptr<pcl_apps_msgs::srv::WritePcd::Response> response) -> void
        {
            (void)request_header;
            if(pointcloud_recieved_)
            {
                int result;
                if(request->format == request->ASCII)
                {
                    result = pcl::io::savePCDFileASCII(request->path, cloud_);
                }
                else if(request->format == request->BINARY)
                {
                    result = pcl::io::savePCDFileBinary(request->path, cloud_);
                }
                else if(request->format == request->BINARY_COMPRESSED)
                {
                    result = pcl::io::savePCDFileBinaryCompressed(request->path, cloud_);
                }
                else
                {
                    response->result = response->FAIL;
                    response->description = "invalid file format";
                }

                if(result == 0)
                {
                    response->result = response->SUCCESS;
                    response->path = request->path;
                    response->description = "Succeed to write PCD";
                }
                else
                {
                    response->result = response->FAIL;
                    response->description = "Failed to write PCD";
                }
            }
            else
            {
                response->result = response->FAIL;
                response->description = "point cloud does not recieved";
            }
        };
        std::string service_name = get_name() + std::string("/write_pcd");
        server_ = create_service<pcl_apps_msgs::srv::WritePcd>(service_name,write_pcd_callback);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::PcdWriterComponent)