cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Uncomment and modify this to add your executable
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp)

# Uncomment this to install your executable
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
