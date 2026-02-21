cmake_minimum_required(VERSION 3.5)
project(ros2_rs_pcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)
find_package(pluginlib REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(rs_pcl_filter_composable SHARED
  src/filter/rspcl_filter_component.cpp
  src/filter/rspcl_filter_node.cpp
)


ament_target_dependencies(rs_pcl_filter_composable
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

rclcpp_components_register_nodes(rs_pcl_filter_composable
  RspclFilterComponent
)

target_link_libraries(rs_pcl_filter_composable
  ${PCL_LIBRARIES}
)


# Executables
add_executable(rs_pcl_filter src/filter/rspcl_filter_node.cpp src/filter/rspcl_filter_component.cpp)
add_executable(rs_pcl_filter_seg src/filter/rspcl_filter_node.cpp src/filter/rspcl_filter_component_segm.cpp)
add_executable(rs_pcl_clustering src/clustering/rspcl_clustering_node.cpp src/clustering/rspcl_clustering_component.cpp)
# add_executable(rs_pcl_marker src/visualization_marker/rspcl_marker.cpp)


pluginlib_export_plugin_description_file(rclcpp_components plugins.xml)



# Apply dependencies to the rs_pcl_clustering target
ament_target_dependencies(rs_pcl_clustering
  rclcpp 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(rs_pcl_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)

ament_target_dependencies(rs_pcl_filter_seg
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

target_link_libraries(rs_pcl_filter
  rs_pcl_filter_composable
  ${PCL_LIBRARIES}
)

target_link_libraries(rs_pcl_filter_seg
  ${PCL_LIBRARIES}
)

target_link_libraries(rs_pcl_clustering ${PCL_LIBRARIES})
# target_link_libraries(rs_pcl_marker ${PCL_LIBRARIES})

install(TARGETS
  rs_pcl_filter
  rs_pcl_filter_seg 
  rs_pcl_clustering
  rs_pcl_filter_composable
  # rs_pcl_marker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
