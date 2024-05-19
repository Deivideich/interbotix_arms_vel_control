import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
import math as m

class interbotix_arm_handler(Node):
    def __init__(self):
        super().__init__('arm_handler')
        self.mode = 'position'
        self.joint_limits_subscriber = self.create_subscription(JointState, '/wx250s/joint_states', self.check_arm_limits, 10)
        self.velocity_publisher = self.create_publisher(JointGroupCommand, '/wx250s/commands/joint_group', 10)
        self.velocity_subscriber = self.create_subscription(JointGroupCommand, '/matlab/velocities', self.callback,10)
        self.waist_velocity = 0.0
        self.shoulder_velocity = 0.0
        self.elbow_velocity = 0.0
        self.forearm_roll_velocity = 0.0
        self.wrist_angle_velocity = 0.0
        self.wrist_rotate_velocity = 0.0
        self.gripper_velocity = 0.0
    
    def set_mode(self, mode):
        self.mode = mode
        req = OperatingModes.Request()
        req.mode = mode
        req.cmd_type = 'group'
        req.name = 'all'
        req.profile_type = 'velocity'
        req.profile_velocity = 131
        req.profile_acceleration = 15
        add_two_ints = self.create_client(OperatingModes, '/wx250s/set_operating_modes')
        while not add_two_ints.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = add_two_ints.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

    def check_arm_limits(self, msg):
        joint_states = msg.position
        for i in range (0, 6):
             if abs(joint_states[i]) > 2*m.pi:
                 self.set_velocity([0.0]*9)
                 break
                 
    #velocities are:
    #1-waist
    #2-shoulder
    #3-elbow
    #4-forearm_roll
    #5-wrist_angle
    #6-wrist_rotate
    #7-gripper
    def set_velocity(self, velocities):
        msg = JointGroupCommand()
        msg.cmd = velocities
        for i in range (0,1):
            velocities.append(0.0)
        msg.name = 'all'
        self.velocity_publisher.publish(msg)

    def callback(self, msg):
        self.velocity_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    interbotix_handler = interbotix_arm_handler()
    interbotix_handler.set_mode('velocity')
    waist_velocity = 0.0
    shoulder_velocity = 0.0
    elbow_velocity = 0.0
    forearm_roll_velocity = 0.0
    wrist_angle_velocity = 0.0
    wrist_rotate_velocity = 0.0
    gripper_velocity = 0.0
    interbotix_handler.set_velocity([waist_velocity,shoulder_velocity,elbow_velocity,forearm_roll_velocity,wrist_angle_velocity,wrist_rotate_velocity,gripper_velocity])
    rclpy.spin(interbotix_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interbotix_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
