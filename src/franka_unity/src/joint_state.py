import random
import rospy
import rosgraph
import time
from sensor_msgs.msg import JointState

TOPIC_NAME = 'joint_states_unity'
NODE_NAME = 'joint_states_unity_publisher'

def joint_state_callback(msg, pub):
    header = msg.header
    name = msg.name
    position = msg.position
    velocity = msg.velocity
    effort = msg.effort

    print("Joint State:")
    print("------------")
    print("{:<10} {:<10} {:<10} {:<10}".format("Name", "Position", "Velocity", "Effort"))
    print("----------------------------------------")

    for i in range(len(name)):
        print("{:<10} {:<10.2f} {:<10.2f} {:<10.2f}".format(name[i], position[i], velocity[i], effort[i]))

    # JointState 정보를 Unity로 전송
    joint_state_array = JointState(header, name, position, velocity, effort)

    wait_for_connections(pub, TOPIC_NAME)
    pub.publish(joint_state_array)
    time.sleep(0.01)


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
    pub = rospy.Publisher(TOPIC_NAME, JointState, queue_size=10)
    rospy.Subscriber('/joint_states', JointState, lambda msg: joint_state_callback(msg, pub))
    rospy.spin()