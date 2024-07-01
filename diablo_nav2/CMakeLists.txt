cmake_minimum_required(VERSION 3.8)
project(diablo_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(data_msgs REQUIRED)
find_package(motion_msgs REQUIRED)
#find_package(diablo_utils REQUIRED)
#find_package(diablo_body REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(dependencies
  rclcpp
  #diablo_body
  motion_msgs
  #diablo_utils
  nav_msgs
  #data_msgs
  geometry_msgs
  #sensor_msgs
  std_msgs
  nav2_msgs
  nav2_util
  PCL
  pcl_ros
  pcl_conversions
)

add_executable(goal_points_generator src/goal_points_generator.cpp)
ament_target_dependencies(goal_points_generator ${dependencies})

install(TARGETS goal_points_generator
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(headings_generator src/headings_generator.cpp)
ament_target_dependencies(headings_generator ${dependencies})

install(TARGETS headings_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  rviz
  config
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
