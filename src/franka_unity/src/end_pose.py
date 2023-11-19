import random
import rospy
import rosgraph
import time
from geometry_msgs.msg import PoseStamped

TOPIC_NAME = 'end_pose_unity'
NODE_NAME = 'end_pose_unity_publisher'

def end_pose_callback(msg, pub):
    header = msg.header
    pose = msg.pose

    # PoseStamped 정보를 Unity로 전송
    end_pose_array = PoseStamped(header, pose)
    
    print("End Pose:", end_pose_array)

    wait_for_connections(pub, TOPIC_NAME)
    pub.publish(end_pose_array)
    time.sleep(0.001)


def wait_for_connections(pub, topic):
    ros_master = rosgraph.Master('/rostopic')
    topic = rosgraph.names.script_resolve_name('rostopic', topic)
    num_subs = 0
    for sub in ros_master.getSystemState()[1]:
        if sub[0] == topic:
            num_subs+=1

    for i in range(10):
        if pub.get_num_connections() == num_subs:
            return
        time.sleep(0.01)
    raise RuntimeError("failed to get publisher")

if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    pub = rospy.Publisher(TOPIC_NAME, PoseStamped, queue_size=10)
    rospy.Subscriber('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, lambda msg: end_pose_callback(msg, pub))
    rospy.spin()