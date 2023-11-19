#pragma once
// pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
//KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <iostream>
#include <tf/tf.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include "geometry_msgs/WrenchStamped.h"
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/ModelStates.h"
//EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unordered_set>
#include <sstream>
#include <eigen_conversions/eigen_msg.h>


//matlogger
#include "matlogger2/matlogger2.h"
