cmake_minimum_required(VERSION 3.8)
project(project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rover_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED) 
find_package(OpenCV REQUIRED)


add_executable(Mars_Rover src/Project.cpp)


target_include_directories(Mars_Rover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(Mars_Rover 
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  rover_interfaces
  cv_bridge
  tf2_ros
  ros_gz_interfaces 
)

# OpenCV
target_link_libraries(Mars_Rover ${OpenCV_LIBRARIES})


install(TARGETS Mars_Rover
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()