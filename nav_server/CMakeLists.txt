cmake_minimum_required(VERSION 3.8)
project(nav_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToMap.action"
  DEPENDENCIES geometry_msgs
)

# Changed executable name to avoid conflict
add_executable(nav_server_node src/nav_server_node.cpp)
target_include_directories(nav_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_server_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(nav_server_node
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_ros
  nav2_msgs
  tf2_geometry_msgs
)

rosidl_target_interfaces(nav_server_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS nav_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
