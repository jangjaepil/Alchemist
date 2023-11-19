cmake_minimum_required(VERSION 2.8.3)
project(hrii_ati)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    cmake_modules
    message_generation
    eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(ati_ros_publisher src/ati_ros_publisher.cpp src/ati_iface.cpp)
target_link_libraries(ati_ros_publisher ${catkin_LIBRARIES})
add_dependencies(ati_ros_publisher ${catkin_EXPORTED_TARGETS})

