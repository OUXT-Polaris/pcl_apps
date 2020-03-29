#include <pcl_apps/clustering/euclidean_clustering/euclidean_clustering_component.h>

namespace pcl_apps
{
EuclideanClusteringComponent::EuclideanClusteringComponent(const rclcpp::NodeOptions & options)
: Node("euclidean_clustering", options)
{
  declare_parameter("input_topic", get_name() + std::string("/input"));
  get_parameter("input_topic", input_topic_);
  declare_parameter("cluster_tolerance", 1.0);
  get_parameter("cluster_tolerance", cluster_tolerance_);
  declare_parameter("min_cluster_size", 1);
  get_parameter("min_cluster_size", min_cluster_size_);
  declare_parameter("max_cluster_size", 10000);
  get_parameter("max_cluster_size", max_cluster_size_);
  set_on_parameters_set_callback(
    [this](const std::vector<rclcpp::Parameter> params) -> rcl_interfaces::msg::SetParametersResult
    {
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
      for (auto param : params) {
        if (param.get_name() == "cluster_tolerance") {
          if (cluster_tolerance_ > 0) {
            cluster_tolerance_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "cluster tolerance must be over 0";
          }
        }
        if (param.get_name() == "min_cluster_size") {
          if (min_cluster_size_ >= 1) {
            min_cluster_size_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "min cluster size must be over 0";
          }
        }
        if (param.get_name() == "max_cluster_size") {
          if (max_cluster_size_ >= 1) {
            max_cluster_size_ = param.as_int();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "max cluster size must be over 0";
          }
        }
      }
      if (!results->successful) {
        results->successful = false;
        results->reason = "";
      }
      return *results;
    }
  );
  std::string output_topic_name = get_name() + std::string("/output");
  pub_ = create_publisher<pcl_apps_msgs::msg::PointCloudArray>(output_topic_name, 10);
  auto callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      pcl_apps_msgs::msg::PointCloudArray clusters;
      clusters.header = msg->header;
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
      pcl::fromROSMsg(*msg, *cloud);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> clustering;
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(cloud);
      clustering.setClusterTolerance(cluster_tolerance_);
      clustering.setMinClusterSize(min_cluster_size_);
      clustering.setMaxClusterSize(max_cluster_size_);
      clustering.setSearchMethod(tree);
      clustering.setInputCloud(cloud);
      clustering.extract(cluster_indices);
      for (auto cluster_itr = cluster_indices.begin(); cluster_itr != cluster_indices.end();
        cluster_itr++)
      {
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pointcloud.width = cluster_itr->indices.size();
        pointcloud.height = 1;
        pointcloud.is_dense = false;
        pointcloud.points.resize(pointcloud.width * pointcloud.height);
        for (size_t i = 0; i < cluster_itr->indices.size(); ++i) {
          double x = cloud->points[cluster_itr->indices[i]].x;
          double y = cloud->points[cluster_itr->indices[i]].y;
          double z = cloud->points[cluster_itr->indices[i]].z;
          double intensity = cloud->points[cluster_itr->indices[i]].intensity;
          pcl::PointXYZI p;
          p.x = x;
          p.y = y;
          p.z = z;
          p.intensity = intensity;
          pointcloud.points[i] = p;
        }
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(pointcloud, pointcloud_msg);
        pointcloud_msg.header = msg->header;
        clusters.cloud.push_back(pointcloud_msg);
      }
      pub_->publish(clusters);
    };
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, 10, callback);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_apps::EuclideanClusteringComponent)
