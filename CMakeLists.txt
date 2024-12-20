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
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(CPPAD_INCLUDE_DIRS /usr/local/include/cppad)
set(CPPAD_LIBRARIES /usr/local/lib/libcppad_lib.so)
set(IPOPT_LIBRARIES /usr/local/lib/libipopt.so)

include_directories(include) 
add_library(mpc SHARED src/mpc.cpp)

# Link IPOPT and CPPAD libraries
target_include_directories(mpc PUBLIC ${CPPAD_INCLUDE_DIRS})
target_link_libraries(mpc Eigen3::Eigen ${IPOPT_LIBRARIES} ${CPPAD_LIBRARIES})

add_executable(mpc_ros2 src/mpc_ros2_node.cpp)
ament_target_dependencies(mpc_ros2 
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(mpc_ros2 mpc)


install(TARGETS mpc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS 
  mpc_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()