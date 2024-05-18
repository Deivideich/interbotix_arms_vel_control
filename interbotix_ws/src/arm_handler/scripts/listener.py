#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from sensor_msgs.msg import JointState

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/wx250s/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()