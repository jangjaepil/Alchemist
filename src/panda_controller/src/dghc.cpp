#include "GHCProjections.hpp"
#include "dghc_controller.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_consistent_generalized_hierarchical_constroller");
  
  ros::AsyncSpinner spinner(10);
  spinner.start();
    
  // enum TaskNum{
  //   // JOINT_POSE=0,
  //   MANIPULATOR_IMPEDANCE,
  //   // OBSTACLE_AVOIDANCE_0,
  //   // OBSTACLE_AVOIDANCE_1,
  //   // OBSTACLE_AVOIDANCE_3,
  //   // OBSTACLE_AVOIDANCE_5,
  //   // JOINT_LIMIT1,
  //   // JOINT_LIMIT2,
  //   // JOINT_LIMIT3,
  //   // JOINT_LIMIT4,
  //   // JOINT_LIMIT5,
  //   // JOINT_LIMIT6,
  //   // JOINT_LIMIT7,
  // };
 
  int DOFsize = 7;
  int numTasks = 1;
  Eigen::VectorXd tasksize;
  tasksize = Eigen::VectorXd::Zero(numTasks);
  tasksize[0] = 6;
  
  dghc_controller force_node;
  force_node.init(numTasks, tasksize, DOFsize);
  

  force_node.run();
  spinner.stop();
 
  return 0;
}

