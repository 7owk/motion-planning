cmake_minimum_required(VERSION 3.10)
project(btraj)

# Default to C++17 (ROS 2 Humble standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)
endif()

# ---- find ROS 2 dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- find Eigen ----
find_package(Eigen3 REQUIRED)

# ---- find qpOASES (built from source, lives in /usr/local by default) ----
find_library(QPOASES_LIBRARY
  NAMES qpOASES
  PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
  REQUIRED
)
find_path(QPOASES_INCLUDE_DIR
  NAMES qpOASES.hpp
  PATHS /usr/local/include /usr/include
  REQUIRED
)
message(STATUS "qpOASES library : ${QPOASES_LIBRARY}")
message(STATUS "qpOASES include : ${QPOASES_INCLUDE_DIR}")

# ---- include directories ----
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIR}
)

# ---- A* library ----
add_library(Astarlib
  src/Asearch.cpp
)
ament_target_dependencies(Astarlib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# ---- astar executable ----
add_executable(astar src/astar.cpp)
target_link_libraries(astar Astarlib)
ament_target_dependencies(astar
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# ---- tf_br executable ----
add_executable(tf_br src/tf_br.cpp)
ament_target_dependencies(tf_br
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- b_traj executable ----
add_executable(b_traj src/b_traj.cpp)
target_link_libraries(b_traj
  ${QPOASES_LIBRARY}
)
ament_target_dependencies(b_traj
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# ---- install ----
install(TARGETS
  Astarlib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  astar
  tf_br
  b_traj
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
