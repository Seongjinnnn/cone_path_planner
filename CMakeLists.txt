cmake_minimum_required(VERSION 3.8)
project(cone_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(cone_sorter src/lib/cone_sorter.cpp)
target_include_directories(cone_sorter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(cone_sorter rclcpp geometry_msgs)

add_library(path_finder src/lib/path_finder.cpp)
target_include_directories(path_finder PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(path_finder rclcpp geometry_msgs)

add_library(path_scorer src/lib/path_scorer.cpp)
target_include_directories(path_scorer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(path_scorer rclcpp geometry_msgs)

add_library(cone_matcher src/lib/cone_matcher.cpp)
target_include_directories(cone_matcher PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(cone_matcher rclcpp geometry_msgs)

add_library(centerline_calculator src/lib/centerline_calculator.cpp)
target_include_directories(centerline_calculator PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(centerline_calculator rclcpp geometry_msgs)

add_executable(path_planner_node src/path_planner_node.cpp)

target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  path_planner_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
  builtin_interfaces
  nav_msgs
)

target_link_libraries(path_planner_node
  cone_sorter
  path_finder
  path_scorer
  cone_matcher
  centerline_calculator
)

install(TARGETS path_planner_node cone_sorter path_finder path_scorer cone_matcher centerline_calculator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
