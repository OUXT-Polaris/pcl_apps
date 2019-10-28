#include <pcl_apps/clustering/euclidean_clustering/euclidean_clustering_component.h>

namespace pcl_apps
{
    EuclideanClusteringComponent::EuclideanClusteringComponent(const rclcpp::NodeOptions & options)
    : Node("euclidean_clustering", options)
    {
        declare_parameter("input_topic",get_name() + std::string("/input"));
        get_parameter("input_topic",input_topic_);
        auto callback =
        [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
            pcl::fromROSMsg(*msg, *cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> clustering;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(cloud);
            clustering.setClusterTolerance(cluster_tolerance_);
            clustering.setMinClusterSize(min_cluster_size_);
            clustering.setMaxClusterSize(max_cluster_size_);
            clustering.setSearchMethod(tree);
            clustering.setInputCloud(cloud);
            clustering.extract(cluster_indices);
        };
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::EuclideanClusteringComponent)