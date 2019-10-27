# Points Concatenate Component/Node
add_library(points_concatenate_component SHARED
  points_concatenate/points_concatenate_component.cpp
)
target_link_libraries(points_concatenate_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_concatenate_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs message_filters)
rclcpp_components_register_nodes(points_concatenate_component 
  "pcl_apps::PointsConcatenateComponent")

add_executable(points_concatenate_node
  points_concatenate/points_concatenate_node.cpp)
target_link_libraries(points_concatenate_node 
  points_concatenate_component ${PCL_LIBRARIES})
ament_target_dependencies(points_concatenate_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs message_filters)

# Points Transform Component/Node
add_library(points_transform_component SHARED
  points_transform/points_transform_component.cpp
)
target_link_libraries(points_transform_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_transform_component
  rclcpp rclcpp_components pcl_conversions tf2_ros sensor_msgs)
rclcpp_components_register_nodes(points_transform_component 
  "pcl_apps::PointsTransformComponent")

add_executable(points_transform_node
  points_transform/points_transform_node.cpp)
target_link_libraries(points_transform_node 
  points_transform_component ${PCL_LIBRARIES})
ament_target_dependencies(points_transform_node
  rclcpp rclcpp_components pcl_conversions tf2_ros sensor_msgs)

# Voxelgrid Filter Component/Node
add_library(voxelgrid_filter_component SHARED
  voxelgrid_filter/voxelgrid_filter_component.cpp
)
ament_target_dependencies(voxelgrid_filter_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)
rclcpp_components_register_nodes(voxelgrid_filter_component
  "pcl_apps::VoxelgridFilterComponent")

add_executable(voxelgrid_filter_node
  voxelgrid_filter/voxelgrid_filter_node.cpp
)
target_link_libraries(voxelgrid_filter_node
voxelgrid_filter_component ${PCL_LIBRARIES})
ament_target_dependencies(voxelgrid_filter_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

# install executables/libs
install(TARGETS 
  points_transform_node
  points_concatenate_node
  voxelgrid_filter_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  points_transform_component
  points_concatenate_component
  voxelgrid_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(points_transform_component)
ament_export_libraries(points_concatenate_component)
ament_export_libraries(voxelgrid_filter_component)