/*
 * Author: Wansoo Kim, Pietro Balatti
 * email: wan-soo.kim@iit.it, pietro.balatti@iit.it
 * 
 *
 * ROS implementation to publish ROS msg received by the ATI F/T interface.
 *
 * Publishers: 
 *   - /hrii/ati_wrench     <geometry_msgs::Wrench>    Sensed F/T values
 *   - /hrii/ati_biasset    <std_msgs::Bool>           True when the bias has been computed
 * 
*/

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <boost/chrono.hpp>
#include <ati_iface.h>
#include <stdlib.h>
#include <dirent.h>

#ifndef M_PI
#define M_PI 3.14159
#endif

#define NANO        1000000000.0
#define DATASIZE    1e5 //before1e7
#include <termios.h>
#include <sys/ioctl.h>

using namespace boost::chrono;

bool bComputeBias;  
geometry_msgs::Vector3  force_b_tmp;
geometry_msgs::Vector3  torque_b_tmp;
std::size_t count;

static inline double exponentialSmoothing(double current_raw_value, double last_smoothed_value, double alpha)
{
    return alpha*current_raw_value + (1-alpha)*last_smoothed_value;
}

void atiServiceCallback(const std_msgs::StringConstPtr& msg)
{
  if(!msg->data.compare("bias")){
    ROS_INFO("COMMAND BIAS SUCCESFULLY CALLED");
    bComputeBias = true;
    force_b_tmp.x  = force_b_tmp.y  = force_b_tmp.z  = 0;
    torque_b_tmp.x = torque_b_tmp.y = torque_b_tmp.z = 0;
    count = 0;
  }else{
    ROS_INFO("NO CMD");
  }
}  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ati_ros_publisher");
  ros::NodeHandle nh;
  ros::Subscriber ati_service_sub_ = nh.subscribe("/hrii/ati_service", 20, atiServiceCallback);
  ros::Publisher ati_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/hrii/ati_wrench", 1000);
  ros::Publisher ati_biasset_pub = nh.advertise<std_msgs::Bool>("/hrii/ati_biasset", 1000);
  geometry_msgs::WrenchStamped msg_wrench, wrench, wrench_tmp;
  geometry_msgs::Vector3  force_b,  force_b_tmp;
  geometry_msgs::Vector3  torque_b, torque_b_tmp;
  std_msgs::Bool bias_status;
  ros::Rate loop_rate(100);

  ati_log_t sample;
  Ati_Sens * atiMaster;
  Eigen::VectorXd sensdata_bias;
  
  atiMaster = new Ati_Sens("192.168.1.1", true);    // MINI-45 (SOFTBOT)
//   atiMaster = new Ati_Sens("192.168.0.140", true);    // MINI-45
//   atiMaster = new Ati_Sens("192.168.0.50", true);    // NANO-17
  
  // Remove bias
  std::size_t num_points; num_points  = 50;
  std::size_t count;  count = 0;
  bComputeBias = false;
  bias_status.data = false;
  
  force_b_tmp.x  = force_b_tmp.y  = force_b_tmp.z  = 0;
  torque_b_tmp.x = torque_b_tmp.y = torque_b_tmp.z = 0;
//   sensdata_bias.setZero(6);

  double alpha; alpha = 0.0;

  usleep(5000);

  while (ros::ok())
  {
    
    //Reading external forces from the ATI sensor
    atiMaster->get_last_sample(sample);
    wrench.wrench.force.x = sample.ft[0];
    wrench.wrench.force.y = sample.ft[1];
    wrench.wrench.force.z = sample.ft[2];
    wrench.wrench.torque.x = sample.ft[3];
    wrench.wrench.torque.y = sample.ft[4];
    wrench.wrench.torque.z = sample.ft[5];
    
    exponentialSmoothing(wrench.wrench.force.x,wrench_tmp.wrench.force.x,alpha);
    exponentialSmoothing(wrench.wrench.force.y,wrench_tmp.wrench.force.y,alpha);
    exponentialSmoothing(wrench.wrench.force.z,wrench_tmp.wrench.force.z,alpha);
    exponentialSmoothing(wrench.wrench.torque.x,wrench_tmp.wrench.torque.x,alpha);
    exponentialSmoothing(wrench.wrench.torque.y,wrench_tmp.wrench.torque.y,alpha);
    exponentialSmoothing(wrench.wrench.torque.z,wrench_tmp.wrench.torque.z,alpha);
    wrench_tmp = wrench;
    
    
//     ROS_INFO_STREAM("====== meas ====== ");
//     ROS_INFO_STREAM("wrench: " << wrench);

    
    if(bComputeBias) 
    {
      if(count < num_points) 
      {
        force_b_tmp.x  = force_b_tmp.x + wrench.wrench.force.x;
        force_b_tmp.y  = force_b_tmp.y + wrench.wrench.force.y;
        force_b_tmp.z  = force_b_tmp.z + wrench.wrench.force.z;

        torque_b_tmp.x = torque_b_tmp.x + wrench.wrench.torque.x;
        torque_b_tmp.y = torque_b_tmp.y + wrench.wrench.torque.y;
        torque_b_tmp.z = torque_b_tmp.z + wrench.wrench.torque.z;
        count++;
      } 
      else 
      {
        force_b_tmp.x = (1/(double)num_points) * force_b_tmp.x;
        force_b_tmp.y = (1/(double)num_points) * force_b_tmp.y;
        force_b_tmp.z = (1/(double)num_points) * force_b_tmp.z;

        torque_b_tmp.x = (1/(double)num_points) * torque_b_tmp.x;
        torque_b_tmp.y = (1/(double)num_points) * torque_b_tmp.y;
        torque_b_tmp.z = (1/(double)num_points) * torque_b_tmp.z;

        force_b  = force_b_tmp;
        torque_b = torque_b_tmp;

        // print_bias();
        ROS_INFO_STREAM("====== bias  (mean) ====== ");
        ROS_INFO_STREAM("F: " << force_b);
        ROS_INFO_STREAM("T: " << torque_b);
        ROS_INFO_STREAM("nbSamples: " << count);
        bComputeBias     = false;
        bias_status.data = true;
//         bias_status.data = true;
      }
    }
    else
    {
      // print_bias();
//         ROS_INFO_STREAM("====== Bias  (mean) ====== ");
//         ROS_INFO_STREAM("F: " << force_b);
//         ROS_INFO_STREAM("T: " << torque_b);

      // remove the bias
      msg_wrench.wrench.force.x = (wrench.wrench.force.x - force_b.x)/1000;
      msg_wrench.wrench.force.y = (wrench.wrench.force.y - force_b.y)/1000;
      msg_wrench.wrench.force.z = (wrench.wrench.force.z - force_b.z)/1000;

      msg_wrench.wrench.torque.x = (wrench.wrench.torque.x - torque_b.x)/1000;
      msg_wrench.wrench.torque.y = (wrench.wrench.torque.y - torque_b.y)/1000;
      msg_wrench.wrench.torque.z = (wrench.wrench.torque.z - torque_b.z)/1000;
      
      if(bias_status.data){
		  msg_wrench.header.stamp = ros::Time::now();
		  msg_wrench.header.frame_id = "ATI-MINI-45";
        ati_wrench_pub.publish(msg_wrench);
        ati_biasset_pub.publish(bias_status);
//         ROS_INFO_STREAM("====== meas ====== ");
//         ROS_INFO_STREAM("wrench: " << msg_wrench);
        }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
