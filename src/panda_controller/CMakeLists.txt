cmake_minimum_required(VERSION 3.0.2)
project(panda_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(GHC_DIR "$ENV{HOME}/alchemist/alchemist_ws/src/panda_controller/include")

find_package (Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pinocchio REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  eigen_conversions
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  )

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    geometry_msgs
    gazebo_msgs
    roscpp
    sensor_msgs
    tf
  DEPENDS 
    system_lib
)


include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/panda_controller
  ${catkin_INCLUDE_DIRS}
  ${GHC_DIR}/GHCP
)

add_library(${PROJECT_NAME}
  src/GHCProjections.cpp
)
add_executable(impedance
  src/impedance.cpp
) 
target_link_libraries(impedance ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES} ${Franka_LIBRARIES} )
add_dependencies(impedance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(hand_guidance
  src/hand_guidance.cpp
  src/GHCProjections.cpp
  src/dghc.cpp
)

target_link_libraries(hand_guidance
  
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  pinocchio::pinocchio
  matlogger2::matlogger2

)


find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


