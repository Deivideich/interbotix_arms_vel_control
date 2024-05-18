#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes, OperatingModesRequest
from sensor_msgs.msg import JointState

class interbotix_xs_modules:
    def __init__(self):
        self.joint_positions = [0] * 9
        rospy.Subscriber('/wx250s/joint_states', JointState, self.joint_states_callback)
        self.velocities_publisher = rospy.Publisher('/wx250s/commands/joint_group', JointGroupCommand, queue_size=10)
        rate = rospy.Rate(10) # 10hz
    def joint_states_callback(self,data):
        self.joint_positions = data.position

    def change_mode(self, mode):
        try:
            set_mode = rospy.ServiceProxy('/wx250s/set_operating_modes', OperatingModes)
            req = OperatingModesRequest()
            req.cmd_type='group'
            req.name = 'arm'
            req.mode = mode
            req.profile_type = 'velocity'
            req.profile_velocity = 131
            req.profile_acceleration = 15
            set_mode(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def set_velocities(self,velocities):
        
        cmd_send = JointSingleCommand('arm',1.0)
        print(cmd_send)
        self.velocities_publisher.publish(cmd_send)

if __name__ == '__main__':
    rospy.init_node('arm_handler')
    interbotix = interbotix_xs_modules()
    interbotix.change_mode('velocity')
    velocities = [1.0]
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): 
        interbotix.set_velocities(velocities)
        rate.sleep()
    rospy.spin()
    
# #!/usr/bin/env python3
# # license removed for brevity
# import rospy
# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass