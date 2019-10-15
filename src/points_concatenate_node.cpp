// Headers in this package
#include <pcl_apps/points_concatenate_component.h>
// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<pcl_apps::PointsConcatenateComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}