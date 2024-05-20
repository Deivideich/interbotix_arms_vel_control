import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
import math as m

class interbotix_arm_handler(Node):
    def __init__(self):
        super().__init__('arm_handler')
        self.mode = 'position'
        self.joint_limits_subscriber = self.create_subscription(JointState, '/wx250s/joint_states', self.check_arm_limits, 10)
        self.velocity_publisher = self.create_publisher(JointGroupCommand, '/wx250s/commands/joint_group', 10)
        self.waist_velocity_subscriber = self.create_subscription(Float32, '/matlab/waist_velocity', self.waist_callback, 10)
        self.shoulder_velocity_subscriber = self.create_subscription(Float32, '/matlab/shoulder_velocity', self.shoulder_callback, 10)
        self.elbow_velocity_subscriber = self.create_subscription(Float32, '/matlab/elbow_velocity', self.elbow_callback, 10)
        self.forearm_roll_velocity_subscriber = self.create_subscription(Float32, '/matlab/forearm_roll_velocity', self.forearm_roll_callback, 10)
        self.wrist_angle_velocity_subscriber = self.create_subscription(Float32, '/matlab/wrist_angle_velocity', self.wrist_angle_callback, 10)
        self.wrist_rotate_velocity_subscriber = self.create_subscription(Float32, '/matlab/wrist_rotate_velocity', self.wrist_rotate_callback, 10)
        self.gripper_velocity_subscriber = self.create_subscription(Float32, '/matlab/gripper_velocity', self.gripper_callback, 10)
        self.waist_velocity = 0.0
        self.shoulder_velocity = 0.0
        self.elbow_velocity = 0.0
        self.forearm_roll_velocity = 0.0
        self.wrist_angle_velocity = 0.0
        self.wrist_rotate_velocity = 0.0
        self.gripper_velocity = 0.0
        
    def waist_callback(self, msg):
        self.waist_velocity = msg.data
        self.set_velocities()
    
    def shoulder_callback(self, msg):
        self.shoulder_velocity = msg.data
        self.set_velocities()

    def elbow_callback(self, msg):
        self.elbow_velocity = msg.data
        self.set_velocities()

    def forearm_roll_callback(self, msg):
        self.forearm_roll_velocity = msg.data
        self.set_velocities()

    def wrist_angle_callback(self, msg):
        self.wrist_angle_velocity = msg.data
        self.set_velocities()

    def wrist_rotate_callback(self, msg):
        self.wrist_rotate_velocity = msg.data
        self.set_velocities()

    def gripper_callback(self, msg):
        self.gripper_velocity = msg.data
        self.set_velocities()
        
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
                 print('Joint limit reached')
                 self.set_velocity([0.0]*7)
    
    def set_velocities(self):
        velocities = [self.waist_velocity, self.shoulder_velocity, self.elbow_velocity, self.forearm_roll_velocity, self.wrist_angle_velocity, self.wrist_rotate_velocity, self.gripper_velocity]
        msg = JointGroupCommand()
        msg.name = 'all'
        msg.cmd = velocities
        self.velocity_publisher.publish(msg)



def main(args=None):
    #velocities are:
    #1-waist
    #2-shoulder
    #3-elbow
    #4-forearm_roll
    #5-wrist_angle
    #6-wrist_rotate
    #7-gripper
    rclpy.init(args=args)
    interbotix_handler = interbotix_arm_handler()
    interbotix_handler.set_mode('velocity')
    msg = JointGroupCommand()
    msg.name = 'all'
    msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    interbotix_handler.velocity_publisher.publish(msg)
    rclpy.spin(interbotix_handler)
    interbotix_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
