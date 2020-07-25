# Euclidean Clustering Component/Node
add_library(euclidean_clustering_component SHARED
euclidean_clustering/euclidean_clustering_component.cpp
)
target_compile_definitions(euclidean_clustering_component PRIVATE "PCL_APPS_EUCLIDEAN_CLUSTERING_BUILDING_DLL")
target_link_libraries(euclidean_clustering_component
  boost_system ${PCL_LIBRARIES})
ament_target_dependencies(euclidean_clustering_component
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

add_executable(euclidean_clustering_node
  euclidean_clustering/euclidean_clustering_node.cpp)
target_link_libraries(euclidean_clustering_node
  euclidean_clustering_component ${PCL_LIBRARIES})
ament_target_dependencies(euclidean_clustering_node
  rclcpp rclcpp_components pcl_conversions sensor_msgs pcl_apps_msgs)

# install executables/libs
install(TARGETS
  euclidean_clustering_node
  DESTINATION lib/pcl_apps
)

install(TARGETS
  euclidean_clustering_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(euclidean_clustering_component)