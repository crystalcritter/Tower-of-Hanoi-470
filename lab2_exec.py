#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([136.99, -89.55, 92.93, -94.11, -92.51, -0.68])

# Hanoi tower location 
#added 21-33 as the other positions for the blocks
Q11 = [132.77*pi/180.0, -60.62*pi/180.0, 100.2*pi/180.0, -132.97*pi/180.0, -89.50*pi/180.0, 0*pi/180.0]
Q12 = [132.91*pi/180.0, -56.13*pi/180.0, 103.8*pi/180.0, -140.79*pi/180.0, -89.43*pi/180.0, 0*pi/180.0]
Q13 = [132.3*pi/180.0, -50.17*pi/180.0, 105.63*pi/180.0, -149.2*pi/180.0, -88.78*pi/180.0, 0*pi/180.0]
Q21 = [138.98*pi/180.0, -65.25*pi/180.0, 107.55*pi/180.0, -134.73*pi/180.0, -90.37*pi/180.0, 0*pi/180.0]
Q22 = [137.74*pi/180.0, -59.72*pi/180.0, 113.28*pi/180.0, -149.36*pi/180.0, -89.28*pi/180.0, 0*pi/180.0]
Q23 = [137.97*pi/180.0, -53.17*pi/180.0, 111.23*pi/180.0, -149.30*pi/180.0, -90.59*pi/180.0, 0*pi/180.0]
Q31 = [144.84*pi/180.0, -66.30*pi/180.0, 110.44*pi/180.0, -137.59*pi/180.0, -89.40*pi/180.0, 0*pi/180.0]
Q32 = [144.90*pi/180.0, -60.17*pi/180.0, 110.43*pi/180.0, -140.62*pi/180.0, -89.58*pi/180.0, 0*pi/180.0]
Q33 = [144.90*pi/180.0, -53.76*pi/180.0, 115.34*pi/180.0, -156.37*pi/180.0, -89.92*pi/180.0, 0*pi/180.0]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix #COMPLETED
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states
        self.analog_in_0_value=0.0 #sets original value to 0
        self.io_subscription = self.create_subscription(IOStates,'/io_and_status_controller/io_states',self.io_state_callback, 10)
        #line above makes a subscription to control  gripper
        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called. #COMPLETE
        """
        self.analog_in_0_value = msg.analog_in_states[0].state

    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    #COMPLETE
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height". 
        #home -> Q[start_tower][start_height] -> suction state -> home -> Q[end_tower][end_height] -> suction off -> start
        self.move_arm(home)
        time.sleep(0.1)

        self.move_arm(Q[start_tower][start_height])
        self.set_io(0,1.0)
        time.sleep(0.2)
        #if it doesn't detect we'll have throw an error
        if self.analog_in_0_value < 2:
            return False
        else:
            self.move_arm(home)
            self.move_arm(Q[end_tower][end_height])
            time.sleep(0.1)
            self.set_io(0,0.0)
            time.sleep(0.1)
        return True

    ############### Your Code End Here ###############


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    start_pos = 0
    end_pos = 0
    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        # Get user input
        input_start_tower = input("Enter start tower <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_start_tower + "\n")
        input_end_tower = input("Enter end tower <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_end_tower + "\n")

        if(input_start_tower == input_end_tower):
            print("Quitting...")
            sys.exit() #same start and end tower
        

        if(int(input_start_tower) == 1):
            start_pos = 1
        elif (int(input_start_tower) == 2):
            start_pos = 2
        elif (int(input_start_tower) == 3):
            start_pos = 3
        elif (int(input_start_tower) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        if(int(input_end_tower) == 1):
            end_pos = 1
        elif (int(input_end_tower) == 2):
            end_pos = 2
        elif (int(input_end_tower) == 3):
            end_pos = 3
        elif (int(input_end_tower) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

        ############## Your Code Start Here ##############
        # TODO: modify the code so that UR3e can move tower accordingly from user input
        mid_pos = 6-start_pos-end_pos

        node.move_arm(home)
        # node.get_logger().info(f'Sending goal 1 ...')

        # if not node.move_arm(Q[0][0]):
        #     node.get_logger().error("Failed to move to goal" + str(Q[0][0]))
        #     break
        node.move_block(start_pos,0,end_pos,2) #potentially check the start 
        node.move_block(start_pos,1,mid_pos,2)
        node.move_block(end_pos,2,mid_pos,1)
        node.move_block(start_pos,2,end_pos,2)
        node.move_block(mid_pos,1,start_pos,2)
        node.move_block(mid_pos,2,end_pos,1)
        node.move_block(start_pos,2,end_pos,0)

        node.set_io(0, 1.0)  # Turn/ on suction
        # Delay to make sure suction cup has grasped the block
        time.sleep(1.0)

        # node.get_logger().info(f'Sending goal 2 ...')
        # if not node.move_arm(Q[1][1]):
        #     node.get_logger().error("Failed to move to goal"+str(Q[1][1]))
        #     break

        #     node.get_logger().info(f'Sending goal 3 ...')
        #     if not node.move_arm(Q[2][2]):
        #         node.get_logger().error("Failed to move to goal"+str(Q[2][2]))
        #         break
        #     start_pos = start_pos - 1
        #     node.set_io(0, 0.0)  # Turn off suction

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
