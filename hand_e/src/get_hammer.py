#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sympy as sp
from time import sleep
from std_srvs.srv import SetBool


class HandEControlNode(Node):
    
    def __init__(self):
        super().__init__('hand_e_control_node')
        
        self.velocity_controller_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gripper = self.create_client(SetBool, 'hand_e/gripper/hand_e_switch')
        while not self.gripper.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.gripper1 = self.create_client(SetBool, 'hand_e/gripper/hand_e_switch1')
        while not self.gripper1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = SetBool.Request()
        

    def inverse_kinematics(self):
        # Publishing parameters
        joint_positions = Float64MultiArray()
        velocity = Float64MultiArray()
        
        velocity.data = [0.0,0.0,0.0,0.0]
        self.velocity_controller_pub.publish(velocity)
                
        # Defining symbols for the DH parameters(a, α, d, Θ)
        Θ_symbols = sp.symbols('Θ1:7')
        a_symbols = sp.symbols('a1:7')
        d_symbols = sp.symbols('d1:7')

        # Pre-defined constants for the Denavit-Hartenberg (DH) parameters for UR10 Robot
        a_values = [0, 0.6127, 0.5716, 0, 0, 0]
        α_values = [-sp.pi/2, 0, 0, -sp.pi/2, sp.pi/2, 0]
        d_values = [0.128, 0, 0, 0.1639, 0.1157, 0.1380]

        # Computing DH transformation matrix
        def dh_transform_matrix(a, α, d, Θ):
            return sp.Matrix([
                [sp.cos(Θ), -sp.sin(Θ)*sp.cos(α), sp.sin(Θ)*sp.sin(α), a*sp.cos(Θ)],
                [sp.sin(Θ), sp.cos(Θ)*sp.cos(α), -sp.cos(Θ)*sp.sin(α), a*sp.sin(Θ)],
                [0, sp.sin(α), sp.cos(α), d],
                [0, 0, 0, 1]
            ])
        
        # Defining the offsets for the joint angles
        Θ_offsets = [0, -sp.pi/2, 0, -sp.pi/2, 0, 0]

        # Getting the transformation matrices for all links
        T = [dh_transform_matrix(a, α, d, Θ) for a, α, d, Θ in zip(a_symbols, α_values, d_symbols, Θ_symbols)]
        
        # Creating a dictionary of the form {symbol: value} to substitute the values in the transformation matrices
        a_dict = {a: value for a, value in zip(a_symbols, a_values)}
        d_dict = {d: value for d, value in zip(d_symbols, d_values)}
        values_dict = {**a_dict, **d_dict}
        
        # Calculating the overall transformation matrix from base to end-effector and displaying the result
        T_final = T[0]
        for mtx_t in T[1:]:
            T_final *= mtx_t

        # Calculate and print Jacobian and its components
        Xp = T_final[:-1, -1].subs(values_dict)
        dif_q = [sp.diff(Xp, Θ_symbol) for Θ_symbol in Θ_symbols]
        T_0_n = [sp.prod(T[:i+1]) for i in range(len(T))]
        z_n = [Ti[:-1, 2] for Ti in T_0_n]
        Jacobian = sp.Matrix.vstack(sp.Matrix.hstack(*dif_q), sp.Matrix.hstack(*z_n))

        # Plotting
        total_time = 20
        t = 0
        dt = total_time/400
        ω = 2*sp.pi/total_time # Angular velocity
        initial_angles = [0.0001, -0.0002, 0.0004, -0.0004, 0.0001, -0.0002]    # Initial joint angles with small offsets
        # initial_angles = [0.000,0.000,0.000,0.000,0.000,0.000]   
        initial_angles = [x + y for x, y in zip(initial_angles, Θ_offsets)]
        q = sp.Matrix(initial_angles).evalf()
        subs_Θ = dict(zip(Θ_symbols, initial_angles))
        joint_positions.data = [0.0001, -0.0002, 0.0004, -0.0004, 0.0001, -0.0002]
        self.joint_position_pub.publish(joint_positions)
        print("Setting initial position")
        sleep(1)
        
        x_data, y_data, z_data = [], [], []
        T_final = T_final.subs(values_dict)
        A = T_final.subs(subs_Θ).evalf()
        # start_config = [0.0,0.2561,1.428]
        # end_config = [1.2,0.2561,0.408]
        direction = 1
        print("Moving Manipulator Arm to Pick up Hammer\n")
        while t <= total_time:
            publish_angles = []
            J_Θs = sp.Matrix(Jacobian.subs(subs_Θ)).doit().evalf()
            A = T_final.subs(subs_Θ).evalf()
            
            vx = 0.5*ω*sp.sin(ω*t+sp.pi/2) * direction
            vz = 0.5*ω*sp.cos(ω*t+sp.pi/2) * direction
            x_dot = sp.Matrix([vx, 0.0, vz, 0.0, 0.0, 0.0]).evalf()  # Fixed end-effector velocity
            
            # vx=(end_config[0]-start_config[0])/20
            # vy=(end_config[1]-start_config[1])/20
            # vz=(end_config[2]-start_config[2])/20
            # x_dot = sp.Matrix([vx, vy, vz, 0.0, 0.0, 0.0]).evalf()  # Fixed end-effector velocity
            
            J_inv = J_Θs.pinv()  # Compute pseudo-inverse
            q_dot = J_inv @ x_dot
            q = q + q_dot * dt  # Update joint variables
            subs_Θ = dict(zip(Θ_symbols, [q[i, 0] for i in range(6)]))
            t += dt
            
            for i in range(6):
                if i==1:
                    publish_angles.append(float(-q[i, 0] + Θ_offsets[i])) # For joint 2, Offset is added to make it move in the correct direction
                elif i==4:
                    publish_angles.append(float(-q[i, 0] + Θ_offsets[i]-sp.pi/2)) # For joint 5, Offset is added to make it move in the correct direction
                else:
                    publish_angles.append(float(q[i, 0] - Θ_offsets[i])) # For all other joints, Offset is subtracted to make it move in the correct direction
                    
            print(f"Joint Angles: {publish_angles}",end='\r')
            joint_positions.data = publish_angles
            self.joint_position_pub.publish(joint_positions)
            
            # Pickin up the hammer
            if t>3:
                self.req.data = True
                self.gripper.call_async(self.req)
                self.gripper1.call_async(self.req)
                t=0+dt # Resetting time to avoid singularities during the manipulator's motion
                sleep(5)
                if direction == -1:
                    break
                direction = -1 # Changing direction of motion to move the manipulator arm back to its initial position
        print("\nHammer Picked Up!")

def main(args=None):
    rclpy.init(args=args)
    controller_node = HandEControlNode()
    controller_node.inverse_kinematics()
    controller_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()