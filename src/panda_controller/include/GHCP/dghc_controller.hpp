#include "GHCProjections.hpp"
#include "panda_controller.h"
#pragma once

//순서대로, taskNumber, a_ii, a_ij
typedef std::vector<std::tuple<int,double,double>> priorityTuples;

class dghc_controller : public GHCProjections{
public:
    dghc_controller();
    
    void arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& jointState);
    void externel_wrench_callback(const geometry_msgs::WrenchStamped& externel_wrench);
    void mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input);
    void priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string);
    void ch_pose_callback(const geometry_msgs::Pose& ch_pose);
    void ur_pose_callback(const geometry_msgs::Pose& ur_pose);
    void net_force_callback(const geometry_msgs::Vector3& net_force);
    
    Eigen::Affine3d KDLFrameToEigenFrame(const KDL::Frame& Frame);
    Eigen::VectorXd calculate_error(Eigen::Affine3d & TF_current, Eigen::Affine3d & TF_desired);
    std::vector<double> getDesiredAlphas(priorityTuples priority);
    void desire_pose_callback(const geometry_msgs::Pose::ConstPtr& desire_pose);
    void getModel();
    void getJacobian();
    void getWrench();
    void setInertia();
    void setPriority();
    void getProjectionM();
    void getProjectedToq();
    int run();

private:
  
   double Q_j[7] = {0.0, -0.7854, 0.0, -2.1, 0.0, 1.5708, 0.7854};// initial pos
   double Qdot[7] = {0,0};
   Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(7);
   Eigen::VectorXd q_dot_vec = Eigen::VectorXd::Zero(7);
   Eigen::MatrixXd M_j_, M_j_up,C_j_;
   Eigen::VectorXd G_j_;
   Eigen::MatrixXd j;
   Eigen::MatrixXd j_dot;
   Eigen::VectorXd vir_force = Eigen::VectorXd::Zero(3);
   Eigen::VectorXd toq = Eigen::VectorXd::Zero(7);
   Eigen::VectorXd toqT = Eigen::VectorXd::Zero(7);
   
   ros::NodeHandle nh;
   ros::Subscriber joint_states;
   ros::Subscriber obstacle_states;
   ros::Subscriber externel_wrench_sub;
   ros::Subscriber ur_pose_sub;
   ros::Subscriber desire_pose_sub;
   ros::Subscriber priority_input_sub;
   ros::Subscriber mode_sub;
   ros::Subscriber mass_sub;
   ros::Subscriber riskfactor_sub;
   ros::Subscriber cooperator_sub;
   ros::Subscriber  net_force_sub;
   ros::Publisher panda_joint_effort_pub;
   ros::Publisher ur_pose_pub;
   ros::Subscriber ch_pose_sub;
   std::string base_link;
   std::string end_link;
   Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
   Eigen::MatrixXd B = Eigen::MatrixXd::Identity(6, 6);
   KDL::Tree tree;
   KDL::Chain chain;
   int joint_size;
   int count = 0;
   double dt = 0.001;
   Eigen::VectorXd current_velocity = Eigen::VectorXd::Zero(6);   
   Eigen::VectorXd pos_error = Eigen::VectorXd::Zero(6);     
   bool flag1 = false;
   bool flag2 = false;
   bool flag3 = false;
   bool flag4 = false;
   bool flag5 = false;
   geometry_msgs::Pose desired_ur_pose;
   Eigen::VectorXd desired_position = Eigen::VectorXd::Zero(3);
   Eigen::Matrix3d desired_matrix = Eigen::MatrixXd::Identity(3,3);
   Eigen::MatrixXd jt;
   Eigen::Affine3d desired_end_effector_TF;
   
   Eigen::MatrixXd jimp = Eigen::MatrixXd::Zero(6,7);
   Eigen::VectorXd wrenchExt = Eigen::VectorXd::Zero(6);
   Eigen::VectorXd wrenchExt_tmp = Eigen::VectorXd::Zero(6);
   Eigen::VectorXd wrenchImp = Eigen::VectorXd::Zero(6);
   Eigen::Vector3d position;
   Eigen::Vector3d vel;
   Eigen::Affine3d end_effector_TF;

   Eigen::VectorXd twist;
   Eigen::MatrixXd h_initTur_past = Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d wTur_past = Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d wTur = Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d wTur_last = Eigen::MatrixXd::Identity(4,4);
        
   Eigen::MatrixXd chRh_init = Eigen::MatrixXd::Identity(3,3);
   Eigen::Matrix4d chTh_past = Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d chTh_init = Eigen::MatrixXd::Identity(4,4); 
   Eigen::Matrix4d h_initTch = Eigen::MatrixXd::Identity(4,4); 
   Eigen::Matrix4d wTh_init = Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d wTh_last = Eigen::MatrixXd::Identity(4,4);  
   Eigen::Matrix4d chTh_rel= Eigen::MatrixXd::Identity(4,4); 
   Eigen::Matrix4d chTh= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d chTb= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d bTh= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d h_pastTb= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d h_initTb= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d bTh_init= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d h_initTh= Eigen::MatrixXd::Identity(4,4);
   Eigen::Matrix4d wTch= Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd ur_pastTh_rel = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd wTh = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd wTh_past = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd wTur_init = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd h_initTw = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd h_initTh_past = Eigen::MatrixXd::Identity(4,4);
   Eigen::MatrixXd desired_wTur = Eigen::MatrixXd::Identity(4,4);     
   Eigen::Matrix3d bRe = Eigen::Matrix3Xd::Identity(3,3);
   Eigen::Matrix3d chRe = Eigen::Matrix3Xd::Identity(3,3);
   Eigen::Matrix3d eRd = Eigen::Matrix3Xd::Identity(3,3);
   Eigen::MatrixXd chRb_e = Eigen::MatrixXd::Identity(6,6);
   Eigen::Matrix3d chRb = Eigen::Matrix3d::Zero(3,3);
   Eigen::MatrixXd bRe_e = Eigen::MatrixXd::Zero(6,6);
   Eigen::Matrix3d wRh_init = Eigen::MatrixXd::Identity(3,3);
   Eigen::VectorXd ur_pastPrel = Eigen::Vector3d::Zero(3);
   Eigen::Matrix3d h_initRh = Eigen::MatrixXd::Identity(3,3);
   Eigen::Matrix3d ur_pastRrel = Eigen::MatrixXd::Identity(3,3);
   Eigen::Vector3d axis_h_initRh;
   Eigen::VectorXd wrenchHaptic = Eigen::VectorXd::Zero(6);
   Eigen::Matrix3d wRch = Eigen::Matrix3d::Identity(3,3);
   Eigen::MatrixXd ur_pastTh_init = Eigen::MatrixXd::Identity(4,4); 
   Eigen::MatrixXd ur_pastRh_init = Eigen::MatrixXd::Identity(3,3);
   int desired_pub_count = 0;

   Eigen::VectorXd Net_force = Eigen::VectorXd::Zero(3);
   Eigen::VectorXd Net_force_tmp = Eigen::VectorXd::Zero(3);  
   std::vector<Eigen::MatrixXd> allProjections;

   int numberOfTasks;
   std::string prevPriorityInputString;
   priorityTuples priorityInput;
   priorityTuples filteredPriority;
   priorityTuples prevFilteredPriority;

   std::vector<double> desiredAlphas;
   std::vector<double> scaleValues;

   bool priorityChanged;
   bool alphaChangeDone;
   int modeInput = 0;


   Eigen::Vector3d relativePosition;
   ros::Time previousTime;

   double transitionTime;

   
};