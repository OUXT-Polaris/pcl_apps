cmake_minimum_required(VERSION 3.5)
project(pcl_apps)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  # For x86_64 architecture, SIMD instruction set is fixed below versions,
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
else()
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)

add_compile_definitions(HUMBLE)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

macro(add_pcl_apps_component module component_name)
  ament_auto_add_library(${component_name}_component SHARED
  src/${module}/${component_name}/${component_name}_component.cpp
  )
  target_compile_definitions(${component_name}_component PUBLIC PCL_APPS_PUBLIC)
  target_link_libraries(${component_name}_component
    boost_system ${PCL_LIBRARIES})

  ament_auto_add_executable(${component_name}_node
    src/${module}/${component_name}/${component_name}_node.cpp)
  target_link_libraries(${component_name}_node
    ${component_name}_component ${PCL_LIBRARIES})
  
  ament_export_libraries(${component_name}_component)

  install(TARGETS ${component_name}_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(TARGETS ${component_name}_node DESTINATION lib/pcl_apps)
endmacro(add_pcl_apps_component)

# Filter Modules
add_pcl_apps_component(filter points_concatenate)
rclcpp_components_register_nodes(points_concatenate_component "pcl_apps::PointsConcatenateComponent")
add_pcl_apps_component(filter points_transform)
rclcpp_components_register_nodes(points_transform_component "pcl_apps::PointsTransformComponent")
add_pcl_apps_component(filter radius_outlier_removal)
rclcpp_components_register_nodes(radius_outlier_removal_component "pcl_apps::RadiusOutlierRemovalComponent")
add_pcl_apps_component(filter crop_hull_filter)
rclcpp_components_register_nodes(crop_hull_filter_component "pcl_apps::CropHullFilterComponent")
add_pcl_apps_component(filter crop_box_filter)
rclcpp_components_register_nodes(crop_box_filter_component "pcl_apps::CropBoxFilterComponent")
add_pcl_apps_component(filter pointcloud_to_laserscan)
rclcpp_components_register_nodes(pointcloud_to_laserscan_component "pcl_apps::PointCloudToLaserScanComponent")
add_pcl_apps_component(filter voxelgrid_filter)
rclcpp_components_register_nodes(voxelgrid_filter_component "pcl_apps::VoxelgridFilterComponent")
add_pcl_apps_component(filter intensity_filter)
rclcpp_components_register_nodes(intensity_filter_component "pcl_apps::IntensityFilterComponent")

# Matching Modules
add_pcl_apps_component(matching ndt_matching)
rclcpp_components_register_nodes(ndt_matching_component "pcl_apps::NdtMatchingComponent")
add_pcl_apps_component(matching ndt_matching_twist_estimator)
rclcpp_components_register_nodes(ndt_matching_twist_estimator_component "pcl_apps::NdtMatchingTwistEstimatorComponent")

# IO Module
add_pcl_apps_component(io pcd_writer)
rclcpp_components_register_nodes(pcd_writer_component "pcl_apps::PcdWriterComponent")
add_pcl_apps_component(io pcd_loader)
rclcpp_components_register_nodes(pcd_loader_component "pcl_apps::PcdLoaderComponent")

# Clustering Module
add_pcl_apps_component(clustering euclidean_clustering)
rclcpp_components_register_nodes(pcd_loader_component "pcl_apps::EuclideanClusteringComponent")

# Projection Module
add_pcl_apps_component(projection pointcloud_projection)
rclcpp_components_register_nodes(pointcloud_projection_component "pcl_apps::PointCloudProjectionComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
