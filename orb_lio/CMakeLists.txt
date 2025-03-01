cmake_minimum_required(VERSION 3.0.2)
project(orb_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread ")

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  imagecloud_msg
  cv_bridge
  tf
)
# set(OpenCV_DIR "/home/wsc/opencv-3.4.13/build")


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)
find_package(DBoW3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${DBoW3_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rosbag roscpp rospy sensor_msgs std_msgs imagecloud_msg
  DEPENDS EIGEN3 PCL DBoW3
  INCLUDE_DIRS include
)

add_library(orb_feature SHARED
  src/orb_feature/ORBextractor.cc
  src/orb_feature/ORB_modify.cc
  src/orb_feature/gms_matcher.cc
)

target_link_libraries(orb_feature ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(fast_gicp OpenMP::OpenMP_CXX)
    endif ()
endif ()
target_link_libraries(fast_gicp
  ${PCL_LIBRARIES}
)
target_include_directories(fast_gicp PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(scantoscan_kitti src/scantoscan_kitti.cpp)
target_link_libraries(scantoscan_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} orb_feature)

add_executable(scantomap_kitti src/scantomap_kitti.cpp)
target_link_libraries(scantomap_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${DBoW3_LIBRARIES} orb_feature fast_gicp)
