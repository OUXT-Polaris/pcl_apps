#include <pcl_apps/io/pcd_writer/pcd_writer_component.h>

namespace pcl_apps
{
    PcdWriterComponent::PcdWriterComponent(const rclcpp::NodeOptions & options)
    : Node("points_transform", options)
    {
        declare_parameter("input_topic",get_name() + std::string("/input"));
        get_parameter("input_topic",input_topic_);
        declare_parameter("save_every_pointcloud",true);
        get_parameter("save_every_pointcloud",save_every_pointcloud_);
        pointcloud_recieved_ = false;
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(*msg,*cloud);
            pointcloud_recieved_ = true;
            if(save_every_pointcloud_)
            {
                
            }
        };
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
    }
}