cmake_minimum_required(VERSION 3.8)
project(jackal_wall_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Executable
add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  main_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
