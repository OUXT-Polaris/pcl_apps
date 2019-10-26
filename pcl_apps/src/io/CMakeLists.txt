# Pcd Writer Component/Node 
add_library(pcd_writer_component SHARED
  pcd_writer/pcd_writer_component.cpp
)
target_link_libraries(pcd_writer_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(pcd_writer_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)
rclcpp_components_register_nodes(pcd_writer_component 
  "pcl_apps::PcdWriterComponent")

add_executable(pcd_writer_node
  pcd_writer/pcd_writer_node.cpp)
target_link_libraries(pcd_writer_node 
  pcd_writer_component ${PCL_LIBRARIES})
ament_target_dependencies(pcd_writer_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

# install executables/libs
install(TARGETS
  pcd_writer_component
  pcd_writer_node
  DESTINATION lib/pcl_apps
)