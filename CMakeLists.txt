cmake_minimum_required(VERSION 3.10)
project(mins_package)

# Suppress Eigen array-bounds warning that breaks libpointmatcher on GCC 13
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-array-bounds")

# Build thirdparty dependencies first
add_subdirectory(thirdparty/libnabo)

# libpointmatcher needs nabo headers - add include path now that nabo target exists
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libnabo)
add_subdirectory(thirdparty/libpointmatcher)

# Build ov_core as subdirectory (skip standalone project()/ament_package())
set(OV_CORE_AS_SUBDIRECTORY ON CACHE INTERNAL "")
# Disable aruco tags - OpenCV 4 API incompatibility (cv::aruco::DetectorParameters::create removed)
set(ENABLE_ARUCO_TAGS OFF CACHE BOOL "Disable aruco tags for OpenCV 4 compatibility" FORCE)
add_subdirectory(thirdparty/open_vins/ov_core)

add_subdirectory(mins)
# mins_data and mins_eval require ROS1 catkin, skip them for ROS2

# Required for ament_package()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Ament packaging
ament_package()