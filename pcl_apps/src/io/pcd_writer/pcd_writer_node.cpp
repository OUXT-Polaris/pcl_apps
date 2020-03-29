// Headers in this package
#include <pcl_apps/io/pcd_writer/pcd_writer_component.h>
// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pcl_apps::PcdWriterComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
