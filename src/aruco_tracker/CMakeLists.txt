cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

# Set the path to the OPenCV 4.5
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(OpenCV 4.10 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui aruco)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(aruco REQUIRED)  # Add aruco dependency

# Explicitly find and link opencv_aruco
find_package(OpenCV REQUIRED COMPONENTS aruco)

# just added_2

# Manually link the aruco library if necessary
find_library(OPENCV_ARUCO_LIB NAMES opencv_aruco PATHS /usr/local/lib)
if(OPENCV_ARUCO_LIB)
  message(STATUS "Found OpenCV aruco: ${OPENCV_ARUCO_LIB}")
  list(APPEND OpenCV_LIBRARIES ${OPENCV_ARUCO_LIB})
else()
  message(FATAL_ERROR "OpenCV aruco library not found!")
endif()


# Print OpenCV version and libraries for debugging
# message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")


#include_directories(
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

# Add the executable for aruco_tracker_node
add_executable(aruco_tracker_node src/aruco_tracker.cpp)
# target_link_libraries(aruco_tracker_node ${OpenCV_LIBRARIES} opencv_aruco)


# Add the executable for udp_aruco_receiver
# add_executable(udp_aruco_receiver src/udp_aruco_receiver.cpp)
# target_link_libraries(udp_aruco_receiver ${OpenCV_LIBRARIES} ${cv_bridge_LIBRARIES})


# Link the necessary libraries after the target is defined

target_link_libraries(aruco_tracker_node ${OpenCV_LIBRARIES} opencv_aruco)

#target_link_libraries(aruco_tracker_node ${OpenCV_LIBRARIES})
target_link_libraries(aruco_tracker_node ${OpenCV_LIBRARIES})


# Specify dependencies for ament

# Link the necessary dependencies
#ament_target_dependencies(arucoTrackerNode
ament_target_dependencies(aruco_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  Eigen3
  #aruco  # Ensure that the aruco library is linked
)

# ament_target_dependencies(udp_aruco_receiver
#  rclcpp
#  std_msgs
#  cv_bridge
#)


 #Install the executable
 install(TARGETS 
 aruco_tracker_node        #arucoTrackerNode
# udp_aruco_receiver
  DESTINATION lib/${PROJECT_NAME}
)


# ament package configuration
ament_package()
