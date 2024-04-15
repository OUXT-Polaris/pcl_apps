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

# Filter Modules
add_subdirectory(src/filter)

# Matching Modules
add_subdirectory(src/matching)

# IO Module
add_subdirectory(src/io)

# Clustering Module
add_subdirectory(src/clustering)

# Projection Module
add_subdirectory(src/projection)

rclcpp_components_register_nodes(points_concatenate_component
  "pcl_apps::PointsConcatenateComponent")

rclcpp_components_register_nodes(points_transform_component
  "pcl_apps::PointsTransformComponent")

rclcpp_components_register_nodes(voxelgrid_filter_component
  "pcl_apps::VoxelgridFilterComponent")

rclcpp_components_register_nodes(ndt_matching_component
  "pcl_apps::NdtMatchingComponent")

rclcpp_components_register_nodes(ndt_matching_twist_estimator_component
  "pcl_apps::NdtMatchingTwistEstimatorComponent")

rclcpp_components_register_nodes(pcd_writer_component
  "pcl_apps::PcdWriterComponent")

rclcpp_components_register_nodes(pcd_loader_component
  "pcl_apps::PcdLoaderComponent")

rclcpp_components_register_nodes(radius_outlier_removal_component
  "pcl_apps::RadiusOutlierRemovalComponent")

rclcpp_components_register_nodes(crop_hull_filter_component
  "pcl_apps::CropHullFilterComponent")

rclcpp_components_register_nodes(crop_box_filter_component
  "pcl_apps::CropBoxFilterComponent")

rclcpp_components_register_nodes(euclidean_clustering_component
  "pcl_apps::EuclideanClusteringComponent")

rclcpp_components_register_nodes(pointcloud_to_laserscan_component
  "pcl_apps::PointCloudToLaserScanComponent")

rclcpp_components_register_nodes(pointcloud_projection_component
  "pcl_apps::PointCloudProjectionComponent")

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
