cmake_minimum_required(VERSION 3.5)
project(rotating_lidar_accumulator)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_library(conversion include/${PROJECT_NAME}/conversion.cpp)
add_library(scan_layer include/${PROJECT_NAME}/scan_layer.cpp)
add_library(point_cloud_buffer include/${PROJECT_NAME}/point_cloud_buffer.cpp)

add_executable(accumulator src/accumulator.cpp)

target_link_libraries(scan_layer conversion)
target_link_libraries(point_cloud_buffer scan_layer)

target_link_libraries(accumulator
  ${catkin_LIBRARIES}  
  point_cloud_buffer
)


target_include_directories(accumulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(accumulator 
  Eigen3 
  rclcpp 
  std_msgs
  geometry_msgs 
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(scan_layer 
  Eigen3 
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(conversion 
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
)


install(TARGETS
  accumulator
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(${PROJECT_NAME}_test 
#     test/main.cpp
#     test/conversion_unittest.cpp
#     test/scan_layer_unittest.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(${PROJECT_NAME}_test
#     Eigen3 
#     rclcpp 
#     geometry_msgs 
#     sensor_msgs
#     std_msgs
#     PCL
#     pcl_conversions
#     pcl_ros
#   )
#   target_link_libraries(${PROJECT_NAME}_test conversion)
#   target_link_libraries(${PROJECT_NAME}_test scan_layer)
# endif()
ament_package()
