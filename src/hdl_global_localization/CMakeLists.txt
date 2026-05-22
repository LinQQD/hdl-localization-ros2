cmake_minimum_required(VERSION 3.16)
project(hdl_global_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetGlobalLocalizationEngine.srv
  srv/SetGlobalMap.srv
  srv/QueryGlobalLocalization.srv
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp
  src/hdl_global_localization_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}"
  ${OpenCV_LIBS}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib OpenMP::OpenMP_CXX)
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "hdl_global_localization::GlobalLocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

add_dependencies(${PROJECT_NAME}_lib ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
