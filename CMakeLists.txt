cmake_minimum_required(VERSION 3.10)
project(mpc_ros2)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


add_library(mpc SHARED
  src/mpc.cpp)
target_include_directories(mpc PUBLIC include)
ament_target_dependencies(mpc 
  tf2_eigen
)

add_executable(mpc_ros2_node src/mpc_ros2_node.cpp)
ament_target_dependencies(mpc 
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)
target_link_libraries(mpc_ros2_node mpc)


install(TARGETS mpc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS 
  mpc_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()