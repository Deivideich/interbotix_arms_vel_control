import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray
import math as m

class interbotix_arm_handler(Node):
    def __init__(self):
        super().__init__('arm_handler')
        self.mode = 'position'
        self.pub = self.create_publisher(JointGroupCommand, '/wx250s/commands/joint_group', 10)
        self.sub = self.create_subscription(JointGroupCommand, '/matlab/velocities', self.callback,10)
    
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

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    interbotix_handler = interbotix_arm_handler()
    interbotix_handler.set_mode('velocity')
    rclpy.spin(interbotix_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interbotix_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
