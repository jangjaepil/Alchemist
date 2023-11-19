#!/usr/bin/env python

import rospy
from trac_ik_python.trac_ik import IK
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import time
import math

# Initial Joint Values
solution = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
temp_solution = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# prev_desired_pose = Pose()
# prev_desired_pose.position = [-0.385746210813522, 0.191708534955978, 0.370735824108124]
# prev_desired_pose.orientation = [0.682632207870483, 0.184390842914581, 0.184883326292038, -0.682518422603607]

robotTopicName = rospy.get_param('/ur_description')
base_link = "world"
tip_link = "wrist_3_link"
ur5_ik_solver = IK(base_link=base_link, tip_link=tip_link, solve_type="Distance",urdf_string=robotTopicName)  # Use the specified timeout

def callback(data):
    
    global solution
    global temp_solution
    global prev_desired_pose

    # # Convert PoseStamped to Pose
    
    ### example mode ###
    # desired_pose = data.pose

    ### demo mode ###
    desired_pose = data
    print("input solution")
    print(temp_solution)    
    solution = ur5_ik_solver.get_ik(temp_solution, desired_pose.position.x, desired_pose.position.y, desired_pose.position.z, desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w)

    # if prev_desired_pose != desired_pose:

    if not solution == None:
        temp_solution = solution
        print("IK So0lution Found:")
        
        # Publish JointState message to Unity.
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joint_state_msg.position = temp_solution
        joint_state_pub.publish(joint_state_msg)

    else:
        print("IK Solution not found! \n")
        print("setting all joint values to previous pose \n")

    # prev_desired_pose = data

    print("=======================================================================")
    print("output solution")
    for i in temp_solution:
        print(math.degrees(i))
    print("=======================================================================")
    for i in temp_solution:
        print(i)
    print("=======================================================================")
    ### example mode ###
    # prev_desired_pose = data.pose
            
    #     ### demo mode ###
    #     prev_desired_pose = data
            
    # else:
    #     # print("Same pose as previous pose, skipping IK")
    #     pass

def end_pose_listener():
    rospy.init_node('end_pose_listener', anonymous=True)
    
    ### example mode ###
    # rospy.Subscriber('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, callback)

    ### demo mode ###
    # rospy.Subscriber('/panda/end_effector_global_pose', Pose, callback)
    rospy.Subscriber('/ur/desired_pose', Pose, callback)
    
    rospy.spin()

if __name__ == '__main__':
    joint_state_pub = rospy.Publisher('/joint_states_unity', JointState, queue_size=1)
    end_pose_listener()