// Headers in this package
#include <pcl_apps/filter/radius_outlier_removal/radius_outlier_removal_component.h>
// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<pcl_apps::RadiusOutlierRemovalComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}