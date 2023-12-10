#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# Define key codes
lin_vel=3.0
ang_vel=3.0

class MoveHandENode(Node):
    
    def __init__(self):
        super().__init__('move_hand_e_node')
        self.end_loc = [0.0,0.0,0.0]
        self.current_loc = [0.0,0.0,0.0]
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.odom_sub = self.create_subscription(PoseStamped, '/odom', self.odom_callback, 10)
        print("\nHand-E Ready to Move!\n\n")
        print("Move to: \n 1. Home position \n 2. Table position")
        inp = int(input("Enter your choice: "))
        if inp == 1:
            self.end_loc = [0.0,0.0,0.0]
        elif inp == 2:
            self.end_loc = [9.8,-1.3,1.25]
        else:
            print("Invalid input!")
            exit()
            
    def odom_callback(self, msg):
        # print("Updating current location!")
        self.current_loc[0] = msg.pose.position.x
        self.current_loc[1] = msg.pose.position.y
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _,_,yaw = euler_from_quaternion(orientation_list)
        self.current_loc[2] = yaw
        # print("Current location: ",self.current_loc)
        # self.move_hand_e()
        
    def move_hand_e(self):
        
        self.velocity = Float64MultiArray()
        
        def is_at_end():
            tolerance = 0.5  # Adjust as needed
            error_x = abs(self.end_loc[0] - self.current_loc[0])
            error_y = abs(self.end_loc[1] - self.current_loc[1])
            print("\nError(Dist): ",error_x,error_y)
            return error_x < tolerance and error_y < tolerance

        def is_at_end_orientation():
            rclpy.spin_once(self)
            tolerance = 0.01  # Adjust as needed])
            error_yaw = self.end_loc[2]- self.current_loc[2]
            return error_yaw < tolerance and error_yaw > 0
        
        def rotate(sign):
            rclpy.spin_once(self)
            vel = ang_vel * sign
            self.velocity.data = [vel,-vel,vel,-vel,5.0]
            self.velocity_pub.publish(self.velocity)
        
        def stop():
            self.velocity.data = [0.0,0.0,0.0,0.0,5.0]
            self.velocity_pub.publish(self.velocity)
            
        while not is_at_end_orientation():
            sign = 1 if self.end_loc[2] > self.current_loc[2] else -1
            rotate(sign)
        stop()
        while not is_at_end():
            rclpy.spin_once(self)
            self.velocity.data = [lin_vel,lin_vel,lin_vel,lin_vel,5.0]
            self.velocity_pub.publish(self.velocity)
        stop()
        print("Reached destination!")

def main(args=None):
    rclpy.init(args=args)
    move_hand_e_node = MoveHandENode()
    move_hand_e_node.move_hand_e()
    rclpy.spin(move_hand_e_node)
    move_hand_e_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()