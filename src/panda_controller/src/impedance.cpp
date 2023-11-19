#include <panda_controller.h>

Eigen::Affine3d KDLFrameToEigenFrame(const KDL::Frame& Frame);
Eigen::VectorXd calculate_error(Eigen::Affine3d & TF_current, Eigen::Affine3d & TF_desired);

class panda_controller
{
public:
  panda_controller()
  {
    joint_state_sub = n.subscribe("/panda/joint_states", 100, &panda_controller::joint_states_callback,this);
    desire_pose_sub = n.subscribe("/panda/desire_pose", 100, &panda_controller::desire_pose_callback,this);


    panda_finger_joint1_position_pub = n.advertise<std_msgs::Float64>("/panda/panda_finger_joint1_position_controller/command", 100);
    panda_finger_joint2_position_pub = n.advertise<std_msgs::Float64>("/panda/panda_finger_joint1_position_controller/command", 100);
    panda_joint_effort_pub = n.advertise<std_msgs::Float64MultiArray>("/panda/panda_joint_effort_controller/command", 100);
    panda_end_effector_pose_pub = n.advertise<geometry_msgs::Pose>("/panda/end_effector_global_pose", 100);
    
    base_link = "world"; // manipulator의 base link 이름으로 변경
    end_link = "panda_link8"; // manipulator의 end link 이름으로 변경

    if (!kdl_parser::treeFromParam("robot_description", tree)) 
    {
        ROS_ERROR("Failed to construct kdl tree");
    }

        // KDL::Chain chain;
    if (!tree.getChain(base_link, end_link, chain)) 
    {
        ROS_ERROR("Failed to get chain from tree");
    }
   

    joint_size = chain.getNrOfJoints();

    B.diagonal() << 5, 5, 5, 0.1, 0.1, 0.1;
    K.diagonal() << 200, 200, 200, 15, 15, 15;

    desired_end_effector_TF.translation() << 0.323895, 0, 0.610663;

    desired_end_effector_TF.linear() <<-0.87157,          0,  0.49027,
                                              0,         -1,        0,
                                        0.49027,          0, -0.87157;


  }
  

  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& JointState)
  {
    JointPosition[0] = JointState->position[0];
    JointPosition[1] = JointState->position[1];
    JointPosition[2] = JointState->position[2];
    JointPosition[3] = JointState->position[3];
    JointPosition[4] = JointState->position[4];
    JointPosition[5] = JointState->position[5];
    JointPosition[6] = JointState->position[6];

    JointVelocity[0] = JointState->velocity[0];
    JointVelocity[1] = JointState->velocity[1];
    JointVelocity[2] = JointState->velocity[2];
    JointVelocity[3] = JointState->velocity[3];
    JointVelocity[4] = JointState->velocity[4];
    JointVelocity[5] = JointState->velocity[5];
    JointVelocity[6] = JointState->velocity[6];

  }

  void desire_pose_callback(const geometry_msgs::Pose::ConstPtr& desire_pose)
  {
    desired_end_effector_TF.translation() << desire_pose->position.x, desire_pose->position.y, desire_pose->position.z;
    Eigen::Quaterniond quaternion(desire_pose->orientation.w, desire_pose->orientation.x, desire_pose->orientation.y, desire_pose->orientation.z);
    desired_end_effector_TF.linear() = quaternion.toRotationMatrix();
  }

  void run()
  {
    ros::Rate loop_rate(100);
    while(ros::ok())
    {   
    
        KDL::JntArray q(joint_size);
       
        KDL::JntArray q_desired(joint_size);
        for (int i = 0; i < joint_size; i++) {
            q(i) = JointPosition[i];
        }


        KDL::JntArray q_dot(joint_size);
        for (int i = 0; i < joint_size; i++) {
            q_dot(i) = JointVelocity[i];
        }

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainJntToJacSolver jac_solver(chain);
        KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));

       
        KDL::Jacobian J(joint_size);
        jac_solver.JntToJac(q, J);

        // Compute the Mass matrix
        KDL::JntSpaceInertiaMatrix M(joint_size);
        dyn_param.JntToMass(q,M);
     
        // Compute the Coriolis term = C(q,q_dot)*q_dot
        KDL::JntArray C(joint_size);
        dyn_param.JntToCoriolis(q,q_dot,C);

        // Compute the Gravity term
        KDL::JntArray G(joint_size);
        dyn_param.JntToGravity(q,G);

        KDL::Frame end_effector_pose;
        fk_solver.JntToCart(q, end_effector_pose);


        Eigen::Affine3d end_effector_TF = KDLFrameToEigenFrame(end_effector_pose);

        inertia_matrix = M.data;
        Jacobian = J.data;
        Coriolis = C.data;
        Gravity = G.data;

        q_d_vec = q_desired.data;
        q_vec = q.data;
        qdot_vec = q_dot.data;

        Eigen::VectorXd current_velocity = Jacobian * qdot_vec;

        Eigen::VectorXd pos_error = calculate_error(end_effector_TF, desired_end_effector_TF);

        // std::cout <<"desired_end_effector_TF" << std::endl << desired_end_effector_TF.matrix() << std::endl;
        // std::cout <<"end_effector_TF" << std::endl << end_effector_TF.matrix() << std::endl;
        // std::cout <<"pos_error" << std::endl << pos_error.transpose() << std::endl;
        // std::cout <<"F_ext" << std::endl << (-K * pos_error - B * current_velocity).transpose() << std::endl;
         
        // tau_control calculation
        Eigen::VectorXd tau_control = Jacobian.transpose() * (-K * pos_error - B * current_velocity) + Gravity;
        
        geometry_msgs::Pose pose_msg;
        
        std_msgs::Float64MultiArray effort;
        for(int i = 0; i < joint_size; i++)
        {
          effort.data.push_back(tau_control(i));
        }

        Eigen::Vector3d translation = end_effector_TF.translation();
        Eigen::Quaterniond rotation(end_effector_TF.rotation());
        pose_msg.position.x = translation.x();
        pose_msg.position.y = translation.y();
        pose_msg.position.z = translation.z();
        pose_msg.orientation.x = rotation.x();
        pose_msg.orientation.y = rotation.y();
        pose_msg.orientation.z = rotation.z();
        pose_msg.orientation.w = rotation.w();
        
        panda_joint_effort_pub.publish(effort);
        panda_end_effector_pose_pub.publish(pose_msg);
  
        
        loop_rate.sleep();
    }
  }

private:
  int joint_size;
  float JointPosition[7] = {0.0, -0.7854, 0.0, -2.1, 0.0, 1.5708, 0.7854};// initial pos error
  float JointVelocity[7] = {0.0};

  ros::NodeHandle n;
  ros::Subscriber joint_state_sub;
  ros::Publisher panda_joint_effort_pub;
  ros::Publisher panda_end_effector_pose_pub;
  ros::Publisher panda_finger_joint1_position_pub;
  ros::Publisher panda_finger_joint2_position_pub;

  ros::Subscriber desire_pose_sub;


  Eigen::VectorXd q_vec; 
  Eigen::VectorXd q_d_vec;
  Eigen::VectorXd qdot_vec;
  Eigen::VectorXd qddot_vec;
  Eigen::VectorXd Coriolis;
  Eigen::VectorXd Gravity;
  Eigen::MatrixXd Jacobian;
  Eigen::MatrixXd inertia_matrix;
  Eigen::VectorXd torque;
  
  Eigen::VectorXd current_velocity;

  Eigen::MatrixXd Mass;


  double dt = 0.01;

  
  std::string base_link;

  std::string end_link;

  KDL::Tree tree;
  KDL::Chain chain;

  Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(6, 6);

  Eigen::Affine3d desired_end_effector_TF;



  unsigned joint_SIZE[6];


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "impedance");
  ros::AsyncSpinner spinner(10);
  spinner.start();

  panda_controller force_node;
  force_node.run();
  
  return 0;
}


Eigen::Affine3d KDLFrameToEigenFrame(const KDL::Frame& Frame)
{
  Eigen::Affine3d affine;
  affine.translation() << Frame.p.x(), Frame.p.y(), Frame.p.z();
  Eigen::Quaterniond q(Frame.M.data);
   std::cout<<"quaternion: "<<q.x() <<q.y() <<q.z() <<q.w()<<std::endl;
  std::cout<<"position: "<< Frame.p.x()<< Frame.p.y()<< Frame.p.z();
  affine.linear() = q.toRotationMatrix();
  return affine;
}

Eigen::VectorXd calculate_error(Eigen::Affine3d & TF_current, Eigen::Affine3d & TF_desired)
{

  // compute error to desired pose
  // position error
  Eigen::Vector3d current_position(TF_current.translation());
  Eigen::Quaterniond current_orientation(TF_current.linear());
  Eigen::Vector3d position_d_(TF_desired.translation());
  Eigen::Quaterniond orientation_d_(TF_desired.linear());

  Eigen::Matrix<double, 6, 1> pos_error;
  pos_error.head(3) << current_position - position_d_;
  
  // orientation error
  if (orientation_d_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(current_orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  pos_error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  

  return pos_error;
}