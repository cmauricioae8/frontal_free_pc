cmake_minimum_required(VERSION 3.5)
project(frontal_free_pc)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED) # For component node


set(dependencies rclcpp sensor_msgs std_msgs tf2 tf2_ros tf2_sensor_msgs geometry_msgs visualization_msgs)

# add_executable(frontal_free_pc src/frontal_free_pc.cpp)
# ament_target_dependencies(frontal_free_pc ${dependencies})

add_library(frontal_free_pc_component SHARED src/component_node.cpp)
ament_target_dependencies(frontal_free_pc_component ${dependencies} rclcpp_components)
rclcpp_components_register_nodes(frontal_free_pc_component "free_pc_ns::FrontalFreePC")


install(TARGETS
  # frontal_free_pc
  frontal_free_pc_component
  DESTINATION lib/${PROJECT_NAME}

  # Needed for composition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

