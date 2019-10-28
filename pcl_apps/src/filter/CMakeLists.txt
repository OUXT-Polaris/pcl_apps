# Points Concatenate Component/Node
add_library(points_concatenate_component SHARED
  points_concatenate/points_concatenate_component.cpp
)
target_compile_definitions(points_concatenate_component PRIVATE "PCL_APPS_POINTS_CONCATENATE_BUILDING_DLL")
target_link_libraries(points_concatenate_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_concatenate_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs message_filters)

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
target_compile_definitions(points_transform_component PRIVATE "PCL_APPS_POINTS_TRANSFORM_BUILDING_DLL")
target_link_libraries(points_transform_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_transform_component
  rclcpp rclcpp_components pcl_conversions tf2_ros sensor_msgs)

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
target_compile_definitions(voxelgrid_filter_component PRIVATE "PCL_APPS_VOXELGRID_FILTER_BUILDING_DLL")
ament_target_dependencies(voxelgrid_filter_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

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