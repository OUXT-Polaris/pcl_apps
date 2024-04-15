# Pcd Writer Component/Node
ament_auto_add_library(pcd_writer_component SHARED
  pcd_writer/pcd_writer_component.cpp
)
target_compile_definitions(pcd_writer_component PRIVATE "PCL_APPS_PCD_WRITER_BUILDING_DLL")
target_link_libraries(pcd_writer_component
  boost_system ${PCL_LIBRARIES})

ament_auto_add_executable(pcd_writer_node
  pcd_writer/pcd_writer_node.cpp)
target_link_libraries(pcd_writer_node
  pcd_writer_component ${PCL_LIBRARIES})

# Pcd Loader Component/Node
ament_auto_add_library(pcd_loader_component SHARED
  pcd_loader/pcd_loader_component.cpp
)
target_compile_definitions(pcd_loader_component PRIVATE "PCL_APPS_PCD_LOADER_BUILDING_DLL")
target_link_libraries(pcd_loader_component
  boost_system ${PCL_LIBRARIES})
ament_auto_add_executable(pcd_loader_node
  pcd_loader/pcd_loader_node.cpp)
target_link_libraries(pcd_loader_node
  pcd_loader_component ${PCL_LIBRARIES})
