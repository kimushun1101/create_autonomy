cmake_minimum_required(VERSION 3.5)
project(roomba_600_driver)

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
find_package(rclcpp_components)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(diagnostic_updater REQUIRED)
find_package(ca_msgs REQUIRED)
find_package(libcreate REQUIRED)

include_directories(
  include
  ${ca_msgs_INCLUDE_DIRS}
  ${libcreate_INCLUDE_DIRS}
)

add_library(teleop_twist_joy SHARED src/teleop_twist_joy.cpp)
target_compile_definitions(teleop_twist_joy PRIVATE "COMPOSITION_BUILD_DLL")
ament_target_dependencies(teleop_twist_joy
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
)
rclcpp_components_register_nodes(teleop_twist_joy "teleop_twist_joy::TeleopTwistJoy")

add_library(create_driver SHARED src/create_driver.cpp)
target_compile_definitions(create_driver PRIVATE "COMPOSITION_BUILD_DLL")
ament_target_dependencies(create_driver
  "ca_msgs"
  "diagnostic_msgs"
  # "diagnostic_updater"
  "geometry_msgs"
  "libcreate"
  "nav_msgs"
  "rclcpp"
  "rclcpp_componets"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
)
rclcpp_components_register_nodes(create_driver "create_driver::CreateDriver")

add_executable(teleop_node
  src/teleop_node.cpp)
target_link_libraries(teleop_node
  teleop_twist_joy)
ament_target_dependencies(teleop_node
  "rclcpp")

add_executable(create_driver_node
  src/create_driver_node.cpp)
target_link_libraries(create_driver_node
  create_driver)
ament_target_dependencies(create_driver_node
  "rclcpp")

add_executable(tf_broadcast src/tf_broadcast.cpp)
ament_target_dependencies(tf_broadcast
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2"
  )

install(TARGETS
  teleop_twist_joy
  create_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  tf_broadcast
  teleop_node
  create_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
