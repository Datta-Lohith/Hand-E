#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios

# Define key codes
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 5
MAX_VEL = 30

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        # For Differential Drive Mechanism without plugin
        self.msg = """
        Control The Hand_E Robot!
        ---------------------------
        Moving around:
             w
        a    s    d
             x
        s : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        wheel_velocities = Float64MultiArray()
        left_vel=0.0
        right_vel=0.0


        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 's':  # Quit
                    left_vel=0.0
                    right_vel=0.0
                elif key == 'w' and left_vel <MAX_VEL and right_vel<MAX_VEL:  # Forward
                    left_vel += LIN_VEL_STEP_SIZE 
                    right_vel += LIN_VEL_STEP_SIZE
                elif key == 'x'and left_vel >-MAX_VEL and right_vel>-MAX_VEL:  # Reverse
                    left_vel -= LIN_VEL_STEP_SIZE 
                    right_vel -= LIN_VEL_STEP_SIZE 
                elif key == 'd' and left_vel in range(-MAX_VEL,MAX_VEL) and right_vel in range(-MAX_VEL,MAX_VEL):  # Right
                    right_vel -= ANG_VEL_STEP_SIZE
                    left_vel += ANG_VEL_STEP_SIZE
                elif key == 'a' and left_vel in range(-MAX_VEL,MAX_VEL) and right_vel in range(-MAX_VEL,MAX_VEL):  # Left
                    left_vel -= ANG_VEL_STEP_SIZE
                    right_vel += ANG_VEL_STEP_SIZE
                
                print(f"Left Wheel Velocity: {left_vel}, Right Wheel Velocity: {right_vel}",end="\r")

                wheel_velocities.data = [left_vel, right_vel,left_vel, right_vel]
                self.wheel_velocities_pub.publish(wheel_velocities)

    def run_diff_drive(self):
        # For differential drive mechanism with plugin
        self.msg = """
        Control The Hand_E Robot!
        ---------------------------
        Moving around:
             w
        a    s    d
             x
        s : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        vel_msg = Twist()
        lin_vel=0.0
        ang_vel=0.0
        
        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':
                    break
                elif key == 's':
                    lin_vel=0.0
                    ang_vel=0.0
                elif key == 'w' and lin_vel <MAX_VEL:  # Forward
                    lin_vel += LIN_VEL_STEP_SIZE
                elif key == 'x'and lin_vel >-MAX_VEL:  # Reverse
                    lin_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd' and ang_vel >-MAX_VEL:  # Right
                    ang_vel -= ANG_VEL_STEP_SIZE
                elif key == 'a' and ang_vel <MAX_VEL:  # Left
                    ang_vel += ANG_VEL_STEP_SIZE
                
                print(f"Linear Velocity: {lin_vel}, Angular Velocity: {ang_vel}",end="\r")
                # Publish the twist message
                vel_msg.linear.x = lin_vel
                vel_msg.linear.y = lin_vel
                vel_msg.angular.z = ang_vel
                
                self.velocity_pub.publish(vel_msg)                  
                    
                    
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    # node.run_diff_drive()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()