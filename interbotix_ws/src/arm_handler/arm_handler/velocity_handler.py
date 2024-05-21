import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
import math as m
import time as t

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
        set_operating_mode = self.create_client(OperatingModes, '/wx250s/set_operating_modes')
        while not set_operating_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = set_operating_mode.call_async(req)
        rclpy.spin_until_future_complete(self, resp)

    def check_currents(self):
        self.get_logger().info('Checking currents')
        req = RegisterValues.Request()
        req.cmd_type = 'group'
        req.name = 'all'
        req.reg = 'Present_Current'
        req.value = 0
        get_currents = self.create_client(RegisterValues, '/wx250s/get_motor_registers')
        while not get_currents.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = get_currents.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Current check response: {resp.result()}')
        else:
            self.get_logger().info('Current check request failed')

    def set_currents(self):
        #Change in order to work for all motors
        self.get_logger().info('Setting currents')
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'waist'
        req.reg = 'Goal_Current'
        req.value = 100
        set_currents = self.create_client(RegisterValues, '/wx250s/set_motor_registers')
        while not set_currents.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = set_currents.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Current set response: {resp.result()}')
        else:
            self.get_logger().info('Current set request failed')

    def set_pwm(self):
        #Change in order to work for all motors
        self.get_logger().info('Setting PWM')
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'waist'
        req.reg = 'Goal_PWM'
        req.value = 100
        set_pwm = self.create_client(RegisterValues, '/wx250s/set_motor_registers')
        while not set_pwm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = set_pwm.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'PWM set response: {resp.result()}')
        else:
            self.get_logger().info('PWM set request failed')

    def set_goal_velocity(self):
        self.get_logger().info('Setting goal velocity')
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'waist'
        req.reg = 'Goal_Velocity'
        req.value = 100
        set_goal_velocity = self.create_client(RegisterValues, '/wx250s/set_motor_registers')
        while not set_goal_velocity.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = set_goal_velocity.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Goal velocity set response: {resp.result()}')
        else:
            self.get_logger().info('Goal velocity set request failed')

    def set_goal_position(self):
        self.get_logger().info('Setting goal position')
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'waist'
        req.reg = 'Goal_Position'
        req.value = 100
        set_goal_position = self.create_client(RegisterValues, '/wx250s/set_motor_registers')
        while not set_goal_position.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = set_goal_position.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Goal position set response: {resp.result()}')
        else:
            self.get_logger().info('Goal position set request failed')

    def check_arm_limits(self, msg):
        joint_states = msg.position
        for i in range (0, 6):
             if abs(joint_states[i]) > 2*m.pi:
                 print('Joint limit reached')
                 self.set_velocities([0.0]*7)

    def check_arm_temperature(self):
        req = RegisterValues.Request()
        req.cmd_type = 'group'
        req.name = 'all'
        req.reg = 'Present_Temperature'
        req.value = 0
        get_temperature = self.create_client(RegisterValues, '/wx250s/get_motor_registers')
        while not get_temperature.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = get_temperature.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Temperature check response: {resp.result()}')
        else:
            self.get_logger().info('Temperature check request failed')

    def check_arm_velocities(self):
        req = RegisterValues.Request()
        req.cmd_type = 'group'
        req.name = 'all'
        req.reg = 'Present_Velocity'
        req.value = 0
        get_velocities = self.create_client(RegisterValues, '/wx250s/get_motor_registers')
        while not get_velocities.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = get_velocities.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Velocity check response: {resp.result()}')
        else:
            self.get_logger().info('Velocity check request failed')
    
    def check_arm_positions(self):
        req = RegisterValues.Request()
        req.cmd_type = 'group'
        req.name = 'all'
        req.reg = 'Present_Position'
        req.value = 0
        get_positions = self.create_client(RegisterValues, '/wx250s/get_motor_registers')
        while not get_positions.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        resp = get_positions.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        if resp.result() is not None:
            self.get_logger().info(f'Position check response: {resp.result()}')
        else:
            self.get_logger().info('Position check request failed')

    def set_velocities(self):
        velocities = [self.waist_velocity, self.shoulder_velocity, self.elbow_velocity, self.forearm_roll_velocity, self.wrist_angle_velocity, self.wrist_rotate_velocity, self.gripper_velocity]
        msg = JointGroupCommand()
        msg.name = 'all'
        msg.cmd = velocities
        self.velocity_publisher.publish(msg)
        t.sleep(0.1)



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
    interbotix_handler.check_arm_positions()
    #interbotix_handler.set_mode('velocity')
    #t.sleep(0.1)
    #msg = JointGroupCommand()
    #msg.name = 'all'
    #msg.cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #t.sleep(0.1)
    #interbotix_handler.velocity_publisher.publish(msg)
    #rclpy.spin(interbotix_handler)
    interbotix_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
