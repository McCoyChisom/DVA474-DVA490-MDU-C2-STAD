cmake_minimum_required(VERSION 3.5)
project(uav_cpp)

# Set the path to the MAVSDK installation directory
set(mavsdk_DIR "${HOME}/home/STAD_mccoychisom/uav_ros2_ws/src/MAVSDK/build/default/src")

# Set the path to the Sincere library directory
set(SINCERE_LIB_DIR "${CMAKE_SOURCE_DIR}/../libs/sincere/lib_sincere.so")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

#find_package(sensor_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(MAVSDK REQUIRED COMPONENTS mavsdk mavsdk_action mavsdk_offboard mavsdk_telemetry)


# Include directories
include_directories(
  include  # This includes the 'include' directory for your headers
  include/uav_cpp  # Ensure 'control.h' can be found
  ${CMAKE_SOURCE_DIR}/../libs/sincere  # Include the Sincere library directory
)

# Check if the Sincere library exists
if(NOT EXISTS ${SINCERE_LIB_DIR})
  message(FATAL_ERROR "Sincere library not found! Path checked: ${SINCERE_LIB_DIR}")
else()
  message(STATUS "Sincere library found at ${SINCERE_LIB_DIR}")
endif()

# Link the Sincere library
add_library(sincere SHARED IMPORTED)
set_target_properties(sincere PROPERTIES IMPORTED_LOCATION ${SINCERE_LIB_DIR})

# Add the executable, including engine.c and other source files
add_executable(uav_control
 # src/uav_mai/engine.c	
   src/uav_main/engine.cpp
 # src/uav_main/control_mavsdk.cpp
  src/uav_main/kinematics.cpp
  src/uav_main/mav_util.cpp
  src/uav_main/PID.cpp
#  src/uav_main/arucoTracker.cpp
  #src/uav_main/offboard_main.cpp
)

# Link the MAVSDK libraries and Sincere library
target_link_libraries(uav_control
  MAVSDK::mavsdk                 # Link with MAVSDK
  rclcpp::rclcpp                 # ROS2 dependency
  rt                             # Link with the real-time library for shared memory
  sincere                        # Link with the Sincere library
)

#target_link_libraries(uav_control
#  MAVSDK::mavsdk                 # Link with MAVSDK
#  MAVSDK::mavsdk_action          # Link with MAVSDK action
#  MAVSDK::mavsdk_offboard        # Link with MAVSDK offboard
#  MAVSDK::mavsdk_telemetry       # Link with MAVSDK telemetry
#  rclcpp::rclcpp                 # ROS2 dependency
#  rt                             # Link with the real-time library for shared memory
#  sincere                        # Link with the Sincere library
#)

# Specify additional dependencies for the executable
ament_target_dependencies(uav_control rclcpp uav_msgs)

# Install the executable to be used with ros2 run
install(TARGETS uav_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


















































