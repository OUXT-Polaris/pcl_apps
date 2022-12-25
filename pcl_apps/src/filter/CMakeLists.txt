# Points Concatenate Component/Node
add_library(points_concatenate_component SHARED
  points_concatenate/points_concatenate_component.cpp
)
target_compile_definitions(points_concatenate_component PRIVATE "PCL_APPS_POINTS_CONCATENATE_BUILDING_DLL")
target_link_libraries(points_concatenate_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_concatenate_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs message_synchronizer)

add_executable(points_concatenate_node
  points_concatenate/points_concatenate_node.cpp)
target_link_libraries(points_concatenate_node
  points_concatenate_component ${PCL_LIBRARIES})
ament_target_dependencies(points_concatenate_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs message_synchronizer)

# Points Transform Component/Node
add_library(points_transform_component SHARED
  points_transform/points_transform_component.cpp
)
target_compile_definitions(points_transform_component PRIVATE "PCL_APPS_POINTS_TRANSFORM_BUILDING_DLL")
target_link_libraries(points_transform_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(points_transform_component
  rclcpp rclcpp_components pcl_conversions tf2_ros sensor_msgs tf2_eigen)

add_executable(points_transform_node
  points_transform/points_transform_node.cpp)
target_link_libraries(points_transform_node
  points_transform_component ${PCL_LIBRARIES})
ament_target_dependencies(points_transform_node
  rclcpp rclcpp_components pcl_conversions tf2_ros sensor_msgs tf2_eigen)

# Voxelgrid Filter Component/Node
add_library(voxelgrid_filter_component SHARED
  voxelgrid_filter/voxelgrid_filter_component.cpp
)
target_compile_definitions(voxelgrid_filter_component PRIVATE "PCL_APPS_VOXELGRID_FILTER_BUILDING_DLL")
target_link_libraries(voxelgrid_filter_component
  ${PCL_LIBRARIES})
ament_target_dependencies(voxelgrid_filter_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

add_executable(voxelgrid_filter_node
  voxelgrid_filter/voxelgrid_filter_node.cpp
)
target_link_libraries(voxelgrid_filter_node
  voxelgrid_filter_component ${PCL_LIBRARIES})
ament_target_dependencies(voxelgrid_filter_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

# Radius Outlier Removal Component/Node
add_library(radius_outlier_removal_component SHARED
  radius_outlier_removal/radius_outlier_removal_component.cpp
)
target_compile_definitions(radius_outlier_removal_component PRIVATE "PCL_APPS_RADIUS_OUTLIER_REMOVAL_BUILDING_DLL")
target_link_libraries(radius_outlier_removal_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(radius_outlier_removal_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

add_executable(radius_outlier_removal_node
  radius_outlier_removal/radius_outlier_removal_node.cpp
)
target_link_libraries(radius_outlier_removal_node
  radius_outlier_removal_component ${PCL_LIBRARIES}
)
ament_target_dependencies(radius_outlier_removal_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs)

# Crop Hull Filter Component/Node
add_library(crop_hull_filter_component SHARED
  crop_hull_filter/crop_hull_filter_component.cpp
)
target_compile_definitions(crop_hull_filter_component PRIVATE "PCL_APPS_CROP_HULL_FILTER_BUILDING_DLL")
target_link_libraries(crop_hull_filter_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(crop_hull_filter_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs message_filters)

add_executable(crop_hull_filter_node
  crop_hull_filter/crop_hull_filter_node.cpp
)
target_link_libraries(crop_hull_filter_node
  crop_hull_filter_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(crop_hull_filter_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs message_filters)

# Crop Box Filter Component/Node
add_library(crop_box_filter_component SHARED
  crop_box_filter/crop_box_filter_component.cpp
)
target_compile_definitions(crop_box_filter_component PRIVATE "PCL_APPS_CROP_HULL_FILTER_BUILDING_DLL")
target_link_libraries(crop_box_filter_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(crop_box_filter_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

add_executable(crop_box_filter_node
  crop_box_filter/crop_box_filter_node.cpp
)
target_link_libraries(crop_box_filter_node
  crop_box_filter_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(crop_box_filter_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

# Point Cloud To Laser Scan Component/Node
add_library(pointcloud_to_laserscan_component SHARED
  pointcloud_to_laserscan/pointcloud_to_laserscan_component.cpp
)
target_compile_definitions(pointcloud_to_laserscan_component PRIVATE "PCL_APPS_POINTCLOUD_TO_LASERSCAN_PUBLIC")
target_link_libraries(pointcloud_to_laserscan_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloud_to_laserscan_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs)
add_executable(pointcloud_to_laserscan_node
  pointcloud_to_laserscan/pointcloud_to_laserscan_node.cpp
)
target_link_libraries(pointcloud_to_laserscan_node
  pointcloud_to_laserscan_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloud_to_laserscan_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

# install executables/libs
install(TARGETS
  points_transform_node
  points_concatenate_node
  voxelgrid_filter_node
  radius_outlier_removal_node
  crop_hull_filter_node
  crop_box_filter_node
  pointcloud_to_laserscan_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  points_transform_component
  points_concatenate_component
  voxelgrid_filter_component
  radius_outlier_removal_component
  crop_hull_filter_component
  crop_box_filter_component
  pointcloud_to_laserscan_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(points_transform_component)
ament_export_libraries(points_concatenate_component)
ament_export_libraries(voxelgrid_filter_component)
ament_export_libraries(radius_outlier_removal_component)
ament_export_libraries(crop_hull_filter_component)
ament_export_libraries(crop_box_filter_component)
