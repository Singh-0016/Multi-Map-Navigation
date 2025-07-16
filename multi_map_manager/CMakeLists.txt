cmake_minimum_required(VERSION 3.8)
project(multi_map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_server REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MultiMapNavigate.action"
  "srv/GetWormhole.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(multi_map_manager_node src/multi_map_manager_node.cpp)

ament_target_dependencies(multi_map_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_server
)


target_include_directories(multi_map_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(multi_map_manager_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


rosidl_target_interfaces(multi_map_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS multi_map_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
