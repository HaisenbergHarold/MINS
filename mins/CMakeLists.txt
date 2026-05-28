cmake_minimum_required(VERSION 3.10)

# We need c++14 for ROS2, thus just require it for everybody
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress unnecessary warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (suppress array-bounds warning due to Eigen+libpointmatcher incompatibility)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wno-array-bounds -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

# Find dependencies
find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED QUIET)
endif()
find_package(Boost REQUIRED QUIET COMPONENTS system filesystem thread date_time program_options chrono)
# libpointmatcher and libnabo are built from thirdparty/ subdirectories (added in parent CMakeLists.txt)
# Targets "nabo" and "pointmatcher" are available from parent scope

# Find ROS2 build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(gnss_msgs REQUIRED)
find_package(message_filters REQUIRED)
# ov_core_lib is built as a subdirectory target (see parent CMakeLists.txt)

# Find PCL
find_package(PCL REQUIRED QUIET)

message(STATUS "OPENCV: ${OpenCV_VERSION} | BOOST: ${Boost_VERSION} | LIBPOINTMATCHER: thirdparty (local)")
message(STATUS "Eigen3: ${Eigen3_VERSION} | PCL: ${PCL_VERSION} | LIBNABO: thirdparty (local)")

add_definitions(-DROS_AVAILABLE=2)
add_definitions(-DROS2)

# Include our header files
include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /aura/install/gnss_msgs/include/gnss_msgs
  /opt/ros/jazzy/include/pcl_conversions
  /opt/ros/jazzy/include/cv_bridge
  /opt/ros/jazzy/include/image_transport
  /opt/ros/jazzy/include/nav_msgs
  /opt/ros/jazzy/include/sensor_msgs
  /opt/ros/jazzy/include/geometry_msgs
  /opt/ros/jazzy/include/visualization_msgs
  /opt/ros/jazzy/include/rclcpp
  /opt/ros/jazzy/include/tf2
  /opt/ros/jazzy/include/tf2_geometry_msgs
  /opt/ros/jazzy/include/tf2_ros
  /opt/ros/jazzy/include/std_msgs
  /opt/ros/jazzy/include/builtin_interfaces
  /opt/ros/jazzy/include/action_msgs
  /opt/ros/jazzy/include/unique_identifier_msgs
  /opt/ros/jazzy/include/ament_index_cpp
  /opt/ros/jazzy/include
  ${CMAKE_SOURCE_DIR}/thirdparty/libpointmatcher
  ${CMAKE_SOURCE_DIR}/thirdparty/libnabo
  ${CMAKE_SOURCE_DIR}/thirdparty/open_vins/ov_core/src
)

# Set link libraries used by all binaries
set(thirdparty_libraries
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pointmatcher
  nabo
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
  src/options/Options.cpp
  src/options/OptionsCamera.cpp
  src/options/OptionsEstimator.cpp
  src/options/OptionsGPS.cpp
  src/options/OptionsIMU.cpp
  src/options/OptionsInit.cpp
  src/options/OptionsLidar.cpp
  src/options/OptionsSimulation.cpp
  src/options/OptionsSystem.cpp
  src/options/OptionsVicon.cpp
  src/options/OptionsWheel.cpp
  src/core/ROSPublisher.cpp
  src/core/ROSSubscriber.cpp
  src/core/ROSHelper.cpp
  src/sim/Simulator.cpp
  src/sim/ConstBsplineSE3.cpp
  src/sim/SimVisualizer.cpp
  src/utils/Print_Logger.cpp
  src/utils/Jabdongsani.cpp
  src/state/State.cpp
  src/state/StateHelper.cpp
  src/state/Propagator.cpp
  src/core/SystemManager.cpp
  src/update/cam/CamTypes.cpp
  src/update/cam/CamHelper.cpp
  src/update/cam/UpdaterCamera.cpp
  src/update/vicon/UpdaterVicon.cpp
  src/update/gps/UpdaterGPS.cpp
  src/update/wheel/UpdaterWheel.cpp
  src/update/lidar/ikd_Tree.cpp
  src/update/lidar/UpdaterLidar.cpp
  src/update/lidar/LidarHelper.cpp
  src/update/lidar/LidarTypes.cpp
  src/update/UpdaterStatistics.cpp
  src/init/Initializer.cpp
  src/init/imu/I_Initializer.cpp
  src/init/imu_wheel/IW_Initializer.cpp
)

file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(mins_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(mins_lib
  ${thirdparty_libraries}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${pcl_ros_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  gnss_msgs::gnss_msgs__rosidl_typesupport_cpp
  ov_core_lib
)
target_include_directories(mins_lib PUBLIC src/)

# Install library
install(TARGETS mins_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Skip install(DIRECTORY src/ ...) for headers to avoid symlink-install recursion
# Headers are already available at compile time via target_include_directories
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

##################################################
# Make binary files!
##################################################

add_executable(subscribe src/run_subscribe.cpp)
target_link_libraries(subscribe mins_lib)
install(TARGETS subscribe
  DESTINATION lib/${PROJECT_NAME}
)

# Copy launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Ament packaging
ament_package()