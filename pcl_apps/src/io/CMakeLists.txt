# Pcd Writer Component/Node 
add_library(pcd_writer_component SHARED
  pcd_writer/pcd_writer_component.cpp
)
target_compile_definitions(pcd_writer_component PRIVATE "PCL_APPS_PCD_WRITER_BUILDING_DLL")
target_link_libraries(pcd_writer_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(pcd_writer_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

add_executable(pcd_writer_node
  pcd_writer/pcd_writer_node.cpp)
target_link_libraries(pcd_writer_node 
  pcd_writer_component ${PCL_LIBRARIES})
ament_target_dependencies(pcd_writer_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

# Pcd Loader Component/Node 
add_library(pcd_loader_component SHARED
  pcd_loader/pcd_loader_component.cpp
)
target_compile_definitions(pcd_loader_component PRIVATE "PCL_APPS_PCD_LOADER_BUILDING_DLL")
target_link_libraries(pcd_loader_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(pcd_loader_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs std_srvs)
add_executable(pcd_loader_node
  pcd_loader/pcd_loader_node.cpp)
target_link_libraries(pcd_loader_node 
  pcd_loader_component ${PCL_LIBRARIES})
ament_target_dependencies(pcd_loader_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs std_srvs)

# install executables/libs
install(TARGETS
  pcd_writer_node
  pcd_loader_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  pcd_writer_component
  pcd_loader_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(pcd_writer_component)
ament_export_libraries(pcd_loader_component)