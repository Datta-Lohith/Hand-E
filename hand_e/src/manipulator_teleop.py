#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
from std_srvs.srv import SetBool

class ManipulatorControl(Node):
    def __init__(self):
        # Manipulator Control Node
        
        super().__init__('manipulator_control_node')
        self.velocity_controller_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Joint angles
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.0
        self.j4 = 0.0
        self.j5 = 0.0
        self.j6 = 0.0       
        self.joint = Float64MultiArray()
        self.gripper = self.create_client(SetBool, 'hand_e/gripper/hand_e_switch')
        self.gripper1 = self.create_client(SetBool, 'hand_e/gripper1/hand_e_switch')
        while not self.gripper.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()
        while not self.gripper1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()
        
        
    def control_gui(self):
        # GUI for controlling the manipulator
        self.vel=Float64MultiArray()
        self.vel.data = [0.0,0.0,0.0,0.0]
        self.velocity_controller_pub.publish(self.vel)
        
        
        def joint1_slider_change(value):
            joint1_label.config(text=f"Joint 1 Value: {value}")
            self.j1 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
        
        def joint2_slider_change(value):
            joint2_label.config(text=f"Joint 2 Value: {value}")
            self.j2 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
            
        def joint3_slider_change(value):
            joint3_label.config(text=f"Joint 3 Value: {value}")
            self.j3 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
            
        def joint4_slider_change(value):
            joint4_label.config(text=f"Joint 4 Value: {value}")
            self.j4 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
            
        def joint5_slider_change(value):
            joint5_label.config(text=f"Joint 5 Value: {value}")
            self.j5 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
            
        def joint6_slider_change(value):
            joint6_label.config(text=f"Joint 6 Value: {value}")
            self.j6 = float(value)
            self.joint.data = [self.j1,self.j2,self.j3,self.j4,self.j5,self.j6]
            self.joint_position_pub.publish(self.joint)
        
        def gripper():
            if toggle.get()==0:
                self.req.data = False
                gripper_label.config(text=f"Gripper: Close")
                print("Gripper: Close")
            else:
                self.req.data = True
                gripper_label.config(text=f"Gripper: Open")
                print("Gripper: Open")
            self.gripper.call_async(self.req)
            self.gripper1.call_async(self.req)
        
        # Create a main window
        root = tk.Tk()
        root.title("Manipulator Teleop")
        
        # Create slider widgets
        joint1_slider = ttk.Scale(root, from_=-3.14, to=3.14,length=400, orient="horizontal", command=joint1_slider_change)
        joint1_slider.pack(padx=20, pady=20)
        joint2_slider = ttk.Scale(root, from_=-1.3, to=1.84,length=400, orient="horizontal", command=joint2_slider_change)
        joint2_slider.pack(padx=20, pady=20)
        joint3_slider = ttk.Scale(root, from_=-3.14, to=3.14,length=400, orient="horizontal", command=joint3_slider_change)
        joint3_slider.pack(padx=20, pady=20)
        joint4_slider = ttk.Scale(root, from_=-3.14, to=3.14,length=400, orient="horizontal", command=joint4_slider_change)
        joint4_slider.pack(padx=20, pady=20)
        joint5_slider = ttk.Scale(root, from_=-3.14, to=3.14,length=400, orient="horizontal", command=joint5_slider_change)
        joint5_slider.pack(padx=20, pady=20)
        joint6_slider = ttk.Scale(root, from_=-3.14, to=3.14,length=400, orient="horizontal", command=joint6_slider_change)
        joint6_slider.pack(padx=20, pady=20)
        
        # Create a button widget
        toggle = tk.IntVar()
        gripper_button = tk.Checkbutton(root, text="Gripper", variable=toggle, command=gripper)
        gripper_button.pack(padx=20, pady=20)
        gripper_label = tk.Label(root, text="Gripper: Close")
        gripper_label.pack()
        
        # Create labels to display the slider value
        joint1_label = tk.Label(root, text="Joint 1 Value: 0")
        joint1_label.pack()
        joint2_label = tk.Label(root, text="Joint 2 Value: 0")
        joint2_label.pack()
        joint3_label = tk.Label(root, text="Joint 3 Value: 0")
        joint3_label.pack()
        joint4_label = tk.Label(root, text="Joint 4 Value: 0")
        joint4_label.pack()
        joint5_label = tk.Label(root, text="Joint 5 Value: 0")
        joint5_label.pack()
        joint6_label = tk.Label(root, text="Joint 6 Value: 0")
        joint6_label.pack()
        
        # Start the main GUI event loop
        root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    manipulator_control_node = ManipulatorControl()
    manipulator_control_node.control_gui()
    rclpy.spin(manipulator_control_node)
    manipulator_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()