cmake_minimum_required(VERSION 2.8.3)
project(extract_im_pc)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
  rosbag
  gmsl_frame_msg
  cmake_modules
  rospy
  rosbag
)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

add_executable(extract_im_pc extract_im_pc.cpp)
target_link_libraries(extract_im_pc ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(extract_pc_filtered extract_pc_filtered.cpp)
target_link_libraries(extract_pc_filtered ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  extract_im_pc
 # extract_pc_filtered
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



