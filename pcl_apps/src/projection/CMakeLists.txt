# Pointcloud Projection Component/Node
add_library(pointcloud_projection_component SHARED
pointcloud_projection/pointcloud_projection_component.cpp
)
target_compile_definitions(pointcloud_projection_component PRIVATE "PCL_APPS_POINTCLOUD_PROJECTION_BUILDING_DLL")
ament_target_dependencies(pointcloud_projection_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs
  perception_msgs image_geometry message_synchronizer pcl_apps_msgs
  tf2_ros tf2_geometry_msgs visualization_msgs color_names)
target_link_libraries(pointcloud_projection_component ${PCL_LIBRARIES})

add_executable(pointcloud_projection_node
  pointcloud_projection/pointcloud_projection_node.cpp
)
ament_target_dependencies(pointcloud_projection_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs
  perception_msgs image_geometry message_synchronizer pcl_apps_msgs
  tf2_ros tf2_geometry_msgs visualization_msgs color_names)
target_link_libraries(pointcloud_projection_node
  pointcloud_projection_component)

  # install executables/libs
install(TARGETS
  pointcloud_projection_node
  DESTINATION lib/pcl_apps)

install(TARGETS
  pointcloud_projection_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(pointcloud_projection_component)
