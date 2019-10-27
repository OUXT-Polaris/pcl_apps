# Ndt Matching Component/Node
add_library(ndt_matching_component SHARED
  ndt_matching/ndt_matching_component.cpp
)
ament_target_dependencies(ndt_matching_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)
rclcpp_components_register_nodes(ndt_matching_component
  "pcl_apps::NdtMatchingComponent")

add_executable(ndt_matching_node
  ndt_matching/ndt_matching_node.cpp
)
target_link_libraries(ndt_matching_node
ndt_matching_component ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matching_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)

# install executables/libs
install(TARGETS 
  ndt_matching_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  ndt_matching_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(ndt_matching_component)