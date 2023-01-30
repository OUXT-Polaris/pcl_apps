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
find_package(ament_cmake REQUIRED)
find_package(color_names REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(message_synchronizer REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(pcl_apps_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)

if(${tf2_eigen_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_TF2_EIGEN_DEPRECATED_HEADER
  )
endif()

if(${tf2_sensor_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_TF2_SENSOR_MSGS_DEPRECATED_HEADER
  )
endif()

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

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
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(pcl_apps_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ndt_omp)
ament_export_include_directories(include)

ament_package()
