# Ndt Matching Component/Node
add_library(ndt_matching_component SHARED
  ndt_matching/ndt_matching_component.cpp
)
target_compile_definitions(ndt_matching_component PRIVATE "PCL_APPS_NDT_MATCHING_BUILDING_DLL")
target_link_libraries(ndt_matching_component
  ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matching_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)

add_executable(ndt_matching_node
  ndt_matching/ndt_matching_node.cpp
)
target_link_libraries(ndt_matching_node
ndt_matching_component ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matching_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)

# Ndt Matching Twist Estimator Component/Node
add_library(ndt_matching_twist_estimator_component SHARED
  ndt_matching/ndt_matching_twist_estimator_component.cpp
)
target_compile_definitions(ndt_matching_twist_estimator_component PRIVATE "PCL_APPS_NDT_MATCHING_TWIST_ESTIMATOR_BUILDING_DLL")
target_link_libraries(ndt_matching_twist_estimator_component
  ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matching_twist_estimator_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)

add_executable(ndt_matching_twist_estimator_node
  ndt_matching/ndt_matching_twist_estimator_node.cpp
)
target_link_libraries(ndt_matching_twist_estimator_node
ndt_matching_twist_estimator_component ${PCL_LIBRARIES})
ament_target_dependencies(ndt_matching_twist_estimator_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs tf2_eigen)

# install executables/libs
install(TARGETS
  ndt_matching_node
  ndt_matching_twist_estimator_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  ndt_matching_component
  ndt_matching_twist_estimator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(ndt_matching_component)
ament_export_libraries(ndt_matching_twist_estimator_component)