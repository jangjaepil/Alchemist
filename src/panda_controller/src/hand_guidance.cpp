#include "panda_controller.h"
#include "dghc_controller.hpp"
#include "GHCProjections.hpp"


dghc_controller::dghc_controller(){
    joint_states = nh.subscribe("/panda/joint_states", 1000, &dghc_controller::arm_joint_states_callback,this);
    externel_wrench_sub = nh.subscribe("/hrii/ati_wrench", 1000, &dghc_controller::externel_wrench_callback,this);
    priority_input_sub = nh.subscribe("/priorityInput", 1000, &dghc_controller::priority_input_callback,this);
    mode_sub  = nh.subscribe("/modeInput", 1000, &dghc_controller::mode_input_callback,this);
    ch_pose_sub  = nh.subscribe("/head_pose", 1000, &dghc_controller::ch_pose_callback,this);
    net_force_sub  = nh.subscribe("/net_force", 1000, &dghc_controller::net_force_callback,this);
    ur_pose_sub  = nh.subscribe("/ur/current_pose", 1000, &dghc_controller::ur_pose_callback,this);
    panda_joint_effort_pub = nh.advertise<std_msgs::Float64MultiArray>("/panda/panda_joint_effort_controller/command", 1000);
    desire_pose_sub = nh.subscribe("/panda/desire_pose", 100, &dghc_controller::desire_pose_callback,this);
    ur_pose_pub = nh.advertise<geometry_msgs::Pose>("/ur/desired_pose",1000);
    base_link = "panda_link0"; // manipulator의 base link 이름으로 변경
    end_link = "panda_link7"; // manipulator의 end link 이름으로 변경

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
   
    
    chRb << 0,-1,0,
            1,0,0,
            0,0,1;
    chRb_e.block(0,0,3,3) = chRb;
    chRb_e.block(3,3,3,3) = chRb;

    chTb.block(0,3,3,1)<<0.3,-0.3,0.5;
    chTb.block(0,0,3,3) << chRb;

    B.diagonal() << 5, 5, 5, 0.1, 0.1, 0.1;
    K.diagonal() << 200, 200, 200, 15, 15, 15;

    desired_end_effector_TF.translation() << 0.323895, 0, 0.610663;

    desired_end_effector_TF.linear() <<-0.87157,          0,  0.49027,
                                              0,         -1,        0,
                                        0.49027,          0, -0.87157;

          
    
   
}    
void dghc_controller::ch_pose_callback(const geometry_msgs::Pose& ch_pose)
{
    wTch.block(0,3,3,1) << ch_pose.position.x,ch_pose.position.y,ch_pose.position.z; 
    Eigen::Quaterniond quat;
    quat.x() = ch_pose.orientation.x;
    quat.y() = ch_pose.orientation.y;
    quat.z() = ch_pose.orientation.z;
    quat.w() = ch_pose.orientation.w;
    wTch.block(0,0,3,3) << quat.toRotationMatrix();
    wRch = wTch.block(0,0,3,3);
    flag1 = 1;
    
}
void dghc_controller::desire_pose_callback(const geometry_msgs::Pose::ConstPtr& desire_pose)
{
    Eigen::VectorXd d_position_tmp = Eigen::VectorXd::Zero(3); //ref : chair frame
    Eigen::VectorXd d_position = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond d_quat_tmp;
    Eigen::Matrix3d R_tmp = Eigen::Matrix3d::Identity(3,3);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(3,3);
    d_quat_tmp.x() = desire_pose->orientation.x;
    d_quat_tmp.y() = desire_pose->orientation.y;
    d_quat_tmp.z() = desire_pose->orientation.z;
    d_quat_tmp.w() = desire_pose->orientation.w;

    R_tmp = d_quat_tmp.toRotationMatrix();
    R =  chRb.inverse()*R_tmp;
    
    Eigen::Quaterniond quat(R);
    d_position_tmp<< desire_pose->position.x, desire_pose->position.y, desire_pose->position.z;
    d_position = chRb.inverse()*d_position_tmp;
    
    
    
    
    
    desired_end_effector_TF.translation() << d_position(0),d_position(1),d_position(2);

    Eigen::Quaterniond quaternion(desire_pose->orientation.w, desire_pose->orientation.x, desire_pose->orientation.y, desire_pose->orientation.z);
    desired_end_effector_TF.linear() = R;
}


void dghc_controller::net_force_callback(const geometry_msgs::Vector3 &net_force)
{
    double scale = 1;
    Net_force_tmp[0] =scale* net_force.x; //ref : world frame
    Net_force_tmp[1] =scale* net_force.y;
    Net_force_tmp[2] =scale* net_force.z;
   
}


void dghc_controller::ur_pose_callback(const geometry_msgs::Pose& ur_pose)
{
    wTur.block(0,3,3,1) << ur_pose.position.x,ur_pose.position.y,ur_pose.position.z; 
    Eigen::Quaterniond quat;
    quat.x() = ur_pose.orientation.x;
    quat.y() = ur_pose.orientation.y;
    quat.z() = ur_pose.orientation.z;
    quat.w() = ur_pose.orientation.w;
    wTur.block(0,0,3,3) << quat.toRotationMatrix();
    flag2 = 1;
}
void dghc_controller::arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& jointState){
    Q_j[0] = jointState->position[0];
    Q_j[1] = jointState->position[1];
    Q_j[2] = jointState->position[2];
    Q_j[3] = jointState->position[3];
    Q_j[4] = jointState->position[4];
    Q_j[5] = jointState->position[5];
    Q_j[6] = jointState->position[6];
  
    Qdot[0] = jointState->velocity[0];
    Qdot[1] = jointState->velocity[1];
    Qdot[2] = jointState->velocity[2];
    Qdot[3] = jointState->velocity[3];
    Qdot[4] = jointState->velocity[4];
    Qdot[5] = jointState->velocity[5];
    Qdot[6] = jointState->velocity[6];
    flag3 = 1;
  
     
}

Eigen::Affine3d dghc_controller::KDLFrameToEigenFrame(const KDL::Frame& Frame)
{
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  KDL::Rotation rotation_matrix = Frame.M;
  Eigen::MatrixXd eigen_rotation_matrix = Eigen::MatrixXd::Zero(3,3);

  affine.translation() << Frame.p.x(), Frame.p.y(), Frame.p.z();
  
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++) 
    {
        eigen_rotation_matrix(i, j) = rotation_matrix(i, j);
    }
  }

  affine.linear() = eigen_rotation_matrix;
    
    
 
  
  return affine;
}
Eigen::VectorXd dghc_controller::calculate_error(Eigen::Affine3d & TF_current, Eigen::Affine3d & TF_desired)
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
void dghc_controller::externel_wrench_callback(const geometry_msgs::WrenchStamped& externel_wrench){
    wrenchExt_tmp(0) = externel_wrench.wrench.force.x;
    wrenchExt_tmp(1) = externel_wrench.wrench.force.y;
    wrenchExt_tmp(2) = externel_wrench.wrench.force.z; 
    wrenchExt_tmp(3) = externel_wrench.wrench.torque.x;
    wrenchExt_tmp(4) = externel_wrench.wrench.torque.y;
    wrenchExt_tmp(5) = externel_wrench.wrench.torque.z;
    flag5 = 1;
   
}

void dghc_controller::priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string){
    std::string input_str = priority_input_string->data;
    std::stringstream ss(input_str);
    std::unordered_set<char> seen_digits; // 중복된 숫자를 검사하기 위한 집합
    if(prevPriorityInputString == priority_input_string->data){
        return;
    }
    priorityInput.clear();
    prevPriorityInputString= priority_input_string->data;

    int numTask = getNumTasks();
    double value;
    while(ss>> value){
        if(value < 0 || value >= numTask){
                std::cout << "Priority input value must be between 0 and " << numTask-1 << std::endl;
                return;
            }

        if (seen_digits.find(value) != seen_digits.end()) {
        std::cout << "Duplicate digit(" << value << ") found in input_str." << std::endl;
        return;
        }

        seen_digits.insert(value);
    }

    //mode_pub에서 받은 priority를 priorityInput 벡터에서 순서대로 저장
    //alpha값은 0으로 설정하여, strict하게 시작 
    std::stringstream ss1(input_str);
    while(ss1>> value){
        priorityInput.emplace_back(std::make_tuple(value, 0,0)); 
    }
}

void dghc_controller::mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input){
    if(mode_input->data == modeInput){
       
        
    }   
    else if(modeInput == 0 && mode_input->data == 1){
       
        
        chTh_init = chTb*bTh;
        wTh_init = wTch*chTh_init;
        h_initTch = chTh_init.inverse();
        wTur_init = wTur;
        std::cout<<"mode 0 -> mode 1"<<std::endl;
       
       
    }
    else if(modeInput==1 && mode_input->data ==0)
    {
       
        wTur_past = wTur;
        std::cout<<"mode 1 -> mode 0"<<std::endl;
       
    }

    modeInput = mode_input->data;
    //std::cout<<"modeInput: "<<modeInput<<std::endl;
    flag4 = 1;
}

void dghc_controller::getModel(){
    //////////////////////////////////////////////// GET M,C,G,J,J_DOT,POSITION,ORIENTATION VELOCITY TERMS  ///////////////////////////////////////////////////////////////////////////////
        
        KDL::JntArray q(joint_size);
        
        
        for (int i = 0; i < joint_size; i++) {
            q(i) = Q_j[i];
        }


        KDL::JntArray q_dot(joint_size);
        for (int i = 0; i < joint_size; i++) {
            q_dot(i) = Qdot[i];
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
        
            
        end_effector_TF = KDLFrameToEigenFrame(end_effector_pose);
     

       
        q_vec = q.data;
        q_dot_vec = q_dot.data;
        M_j_ = M.data;
        C_j_ = C.data;
        G_j_ = G.data;
        j = J.data;
        jimp << j; //basse to end jacobian
        jt = j.block(0,0,3,7);
        
        position = end_effector_TF.translation();
        current_velocity = jimp * q_dot_vec;
        bRe = end_effector_TF.linear();
        chRe = chRb*bRe;
        // std::cout<<"position: "<<std::endl<<position.transpose()<<std::endl;
        // std::cout<<"bRe: "<<std::endl<<bRe<<std::endl;

        bTh.block(0,3,3,1) = position;
        bTh.block(0,0,3,3) = bRe;
        
         if(count==0)
        {
                std::cout<<"wTur_init: "<<std::endl<<wTur<<std::endl;
                
                
                if(modeInput ==1)
                {   
                    chTh_init = chTb*bTh;
                    wTh_init = wTch*chTh_init;
                    h_initTch = chTh_init.inverse();
                    wTur_past = wTur;
                    wTur_last = wTur;
                    desired_wTur = wTur;
                    
                }
                else
                {
                    wTur_past = wTur;
                    wTur_last = wTur;
                    desired_wTur = wTur;

                }
        }
        else
       {
       
           if(modeInput == 1)
            {
                chTh = chTb*bTh; //done
                h_initTh = h_initTch*chTh; //done
                
               
                ur_pastTh_init = wTur_past.inverse()*wTh_init;
                ur_pastRh_init = ur_pastTh_init.block(0,0,3,3);
                
                wRh_init = wTh_init.block(0,0,3,3);
                ur_pastPrel = ur_pastRh_init * h_initTh.block(0,3,3,1);
                h_initRh = h_initTh.block(0,0,3,3);
                
                Eigen::AngleAxisd angleAxis(h_initRh);
                axis_h_initRh = angleAxis.axis();
                double angle = angleAxis.angle();
                
                Eigen::Vector3d axis_ur_pastRrel;
                axis_ur_pastRrel = ur_pastRh_init*axis_h_initRh;
                
                axis_ur_pastRrel.normalize();
    
                // Create a rotation matrix from axis-angle
                Eigen::AngleAxisd angleAxis2(angle, axis_ur_pastRrel);
                Eigen::Matrix3d ur_pastRrel = angleAxis2.toRotationMatrix();
                
                ur_pastTh_rel.block(0,3,3,1) = ur_pastPrel;
                ur_pastTh_rel.block(0,0,3,3) = ur_pastRrel;
    
                desired_wTur = wTur_past*ur_pastTh_rel;
                wTur_last = desired_wTur;
            }
            else
            {
               desired_wTur = wTur_last;
            }
        
       }
       
       std::cout<<"wTur_past: "<<std::endl<<wTur_past<<std::endl;
       std::cout<<"h_initTh: "<<std::endl<<h_initTh<<std::endl;
       std::cout<<"ur_pastTh_rel: "<<std::endl<<ur_pastTh_rel<<std::endl;
      
               
       desired_position = desired_wTur.block(0,3,3,1);
       desired_matrix = desired_wTur.block(0,0,3,3);
       Eigen::Quaterniond desired_quat(desired_matrix);
       
       desired_ur_pose.position.x = desired_position(0);
       desired_ur_pose.position.y = desired_position(1);
       desired_ur_pose.position.z = desired_position(2); 
       desired_ur_pose.orientation.x = desired_quat.x();
       desired_ur_pose.orientation.y = desired_quat.y();
       desired_ur_pose.orientation.z = desired_quat.z();
       desired_ur_pose.orientation.w = desired_quat.w();
       
       desired_pub_count = desired_pub_count +1;
       if((desired_pub_count % 100) ==99)
       {
         ur_pose_pub.publish(desired_ur_pose);
         desired_pub_count = 0;
       }
       
       
       
       if(count == 0) count = 1;
       
}

void dghc_controller::getJacobian()
{
   std::vector<Eigen::MatrixXd> allJacobians;
   allJacobians.push_back(jimp); 
  
   setJacobianMatrices(allJacobians);
//    std::cout<<"alljacobians:"<<std::endl<<allJacobians[0]<<std::endl;

}

void dghc_controller::getWrench()
{
    bRe_e.block(0,0,3,3) = bRe;
    bRe_e.block(3,3,3,3) = bRe;
   
    wrenchExt = bRe_e*wrenchExt_tmp; //hand guidacne
    
    
    
    pos_error = calculate_error(end_effector_TF, desired_end_effector_TF);
    wrenchImp = - K * pos_error - B * current_velocity;
    



    if(modeInput == 1)
    {
        Net_force = chRb.inverse()*wRch.inverse()*Net_force_tmp;
        //std::cout<<"net_force: "<<std::endl<<Net_force_tmp.transpose()<<std::endl;
    }
    else
    {
        Net_force = Eigen::Vector3d::Zero(3);
    }




    wrenchHaptic[0] = wrenchImp[0] + Net_force[0];
    wrenchHaptic[1] = wrenchImp[1] + Net_force[1];
    wrenchHaptic[2] = wrenchImp[2] + Net_force[2];
    wrenchHaptic[3] = wrenchImp[3] ;
    wrenchHaptic[4] = wrenchImp[4] ;
    wrenchHaptic[5] = wrenchImp[5] ;

}

void dghc_controller::setInertia()
{
   
   setInertiaMatrix(M_j_.inverse());
    // std::cout<<"W: "<<std::endl<<Weighted_wholeM<<std::endl;

}

void dghc_controller::setPriority()
{

    std::vector<double> prioritiesVector;
    for(int row=0; row<numberOfTasks; row++){
        for(int col=row; col<numberOfTasks;col++){
            prioritiesVector.emplace_back(getAlphaIJ(row,col));
        }
    }
    prioritiesVector[0] = 0;
    
    

    int counter=0;
    for(unsigned int i=0; i<numberOfTasks; i++){
        for(unsigned int j=i; j<numberOfTasks; j++){
            if(prioritiesVector[counter] < 0){
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " < 0" << std::endl;
                return;
            }
            if(prioritiesVector[counter] > 1){
                std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector[counter] << " > 1" << std::endl;
                return;
            }
            setAlphaIJ(i,j,prioritiesVector[counter]);
            counter++;
        }
    }
    assert(counter==prioritiesVector.size());
}

void dghc_controller::getProjectionM()
{
    int numTasks = getNumTasks();
    int DOFsize = getDOFsize();
    // std::cout<<"numTasks: "<<numTasks<<std::endl;
    // std::cout<<"DOFsize: "<<DOFsize<<std::endl;
    allProjections.clear();
    for(unsigned int i=0; i<numTasks; i++){
        allProjections.push_back(Eigen::MatrixXd::Zero(DOFsize,DOFsize));
    }
  
    Eigen::VectorXd ranks;
    ranks = Eigen::VectorXd::Zero(numTasks);
    //  std::cout<<"ranks: " <<std::endl<< ranks.transpose()<<std::endl;
     
    bool ok = getAllGeneralizedProjectors(allProjections, ranks);
}

void dghc_controller::getProjectedToq()
{
//    std::cout<<"allprojection0: " <<std::endl<< allProjections[0]<<std::endl;

    toq = allProjections[0]*jimp.transpose()*wrenchHaptic; 
    
    // std::cout<<"p: " <<std::endl<<allProjections[0]<<std::endl;
    // std::cout<<"jimp: " <<std::endl<<jimp<<std::endl;
    // std::cout<<"wrench: " <<std::endl<<wrenchExt<<std::endl;
    if(isnan(toq(0))) toq = Eigen::VectorXd::Zero(7,1);
}    


int dghc_controller::run(){
    numberOfTasks = getNumTasks();
    count = 0;
    ros::Rate loop_rate(1000);
    std::cout<<"waiting callbacks..."<<std::endl;
    while(ros::ok())
    {    
       
       ros::spinOnce();
       
       if(flag1 ==1 && flag2==1 && flag3==1 && flag4==1 && flag5==1)
       { 
           
           
            getModel();
            if(count ==1)
            {
                std::cout<<"start!"<<std::endl;
                count=2;
            }
            getWrench();

            getJacobian();

            setPriority();

            setInertia();

            getProjectionM();

            getProjectedToq();


            toqT = toq + C_j_+ G_j_;
            //std::cout<<"toqT: "<<toqT.transpose()<<std::endl; 
            std_msgs::Float64MultiArray effort;
            for(int i = 0; i < joint_size; i++)
            {
                effort.data.push_back(toqT(i));
            }

            panda_joint_effort_pub.publish(effort);
        
       }
        
        loop_rate.sleep();
    }
    return 0;
}