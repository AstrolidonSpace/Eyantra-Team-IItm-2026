cmake_minimum_required(VERSION 3.8)
project(swift_pico)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)  # ROS2 C++ client library
find_package(rclcpp_action      REQUIRED)  # ROS2 C++ action library
find_package(Eigen3             REQUIRED)  # Linear algebra
find_package(swift_msgs         REQUIRED)  # Drone command msg
find_package(controller_msg     REQUIRED)  # PID Tuning msg
find_package(error_msg          REQUIRED)  # Error visualization msg
find_package(whycode_interfaces REQUIRED)  # WhyCode interfaces
find_package(std_msgs           REQUIRED)  # Header, for /controller_heartbeat
find_package(geometry_msgs      REQUIRED)  # Poses and arrays
find_package(waypoint_navigation REQUIRED) # Custom action and service interfaces

# --- mujoco_bridge (pre-built executable) ------------------------------------
install(
  PROGRAMS config/mujoco_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES config/libmujoco.so.3.9.0
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch and config folders ------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES config/fastdds_profile.xml
  DESTINATION share/${PROJECT_NAME}/config
)

# Looked up at runtime by waypoint_service.py via get_package_share_directory.
install(
  FILES src/grid_whycode_coordinates.json
  DESTINATION share/${PROJECT_NAME}/scripts
)

# --- C++ node: pico_controller_PID --------------------------------------------
add_executable(pico_controller_PID src/pico_controller_PID.cpp)
ament_target_dependencies(pico_controller_PID
  rclcpp
  swift_msgs
  controller_msg
  error_msg
  whycode_interfaces
  std_msgs
)

# --- C++ node: waypoint_service_cpp -------------------------------------------
add_executable(waypoint_service_cpp src/waypoint_service.cpp)
ament_target_dependencies(waypoint_service_cpp
  rclcpp
  geometry_msgs
  waypoint_navigation
)

# --- C++ node: pico_client_cpp ------------------------------------------------
add_executable(pico_client_cpp src/pico_client.cpp)
ament_target_dependencies(pico_client_cpp
  rclcpp
  rclcpp_action
  geometry_msgs
  waypoint_navigation
)

# --- C++ node: pico_server_cpp ------------------------------------------------
add_executable(pico_server_cpp src/pico_server.cpp)
ament_target_dependencies(pico_server_cpp
  rclcpp
  rclcpp_action
  swift_msgs
  error_msg
  whycode_interfaces
  std_msgs
  waypoint_navigation
)

# Install all C++ executables
install(
  TARGETS
    pico_controller_PID
    waypoint_service_cpp
    pico_client_cpp
    pico_server_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# --- Python example: pico_controller_PID.py -----------------------------------
install(
  PROGRAMS src/pico_controller_PID.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install original Python boilerplate scripts
install(
  PROGRAMS
    src/pico_server.py
    src/pico_client.py
    src/waypoint_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
