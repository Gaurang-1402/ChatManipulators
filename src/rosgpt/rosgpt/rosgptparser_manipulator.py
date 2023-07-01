import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
from sensor_msgs.msg import JointState
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import random

class ManipulatorController(Node):

    def __init__(self):
        super().__init__('manipulator_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        
        self.joint_states = JointState()
        self.joint_states.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.thread_executor = ThreadPoolExecutor(max_workers=1)
        self.move_executor = SingleThreadedExecutor()

        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()

        print('ROSGPT Manipulator Controller Started. Waiting for input commands ...')
    
    def joint_state_callback(self, msg):
        self.joint_states = msg

    def center(self):

        time_interval = 0.1/100
        start_time = time.time()
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    
        # Iterate until 5 seconds have passed
        while time.time() - start_time < 5.0:
            # Set the joint positions to center

            # Update the timestamp
            now = time.time()
            self.joint_states.header.stamp.sec = int(now)
            self.joint_states.header.stamp.nanosec = int((now - int(now)) * 1e9)

            # Publish the joint states
            self.joint_state_publisher.publish(self.joint_states)

            # Sleep for the calculated time interval
            time.sleep(time_interval)




    def random(self):
        time_interval = 0.1/100
        start_time = time.time()
        self.joint_states.position = [random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0), random.uniform(-2.99, 2.99), random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0)]
    
        # Iterate until 5 seconds have passed
        while time.time() - start_time < 5.0:
            # Set the joint positions to center

            # Update the timestamp
            now = time.time()
            self.joint_states.header.stamp.sec = int(now)
            self.joint_states.header.stamp.nanosec = int((now - int(now)) * 1e9)

            # Publish the joint states
            self.joint_state_publisher.publish(self.joint_states)

            # Sleep for the calculated time interval
            time.sleep(time_interval)




    def voice_cmd_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json']) 
            print('JSON command received: \n',cmd,'\n')

            if cmd['action'] == 'center':
                self.thread_executor.submit(self.center)

            elif cmd['action'] == 'random':
                self.thread_executor.submit(self.random)

            elif cmd['action'] == 'move_joint':
                joint = cmd['params'].get('joint', None)
                angle = cmd['params'].get('angle', None)
                joint_velocity = cmd['params'].get('speed', None)

                if joint is not None and angle is not None:
                    print(f'joint: {joint}, angle: {angle}')
                    self.thread_executor.submit(self.move_joint, joint, angle)

        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))
    


    def get_distance(self, start, destination):
        return math.sqrt(
            ((destination.position.x - start.position.x) ** 2) +
            ((destination.position.y - start.position.y) ** 2) +
            ((destination.position.z - start.position.z) ** 2)
        )

    def move_joint(self, joint, angle):
        print(f'Start moving the joint {joint} to the angle {angle} radians')
        
        # Validate joint
        if joint not in self.joint_states.name:
            print('[ERROR]: Invalid joint name!')
            return -1
        
        # Validate angle
        if joint == "elbow_joint" and (angle < -2.99 or angle > 2.99):
            print('[ERROR]: Angle out of range for elbow_joint!')
            return -1
        elif (angle < -6.13 or angle > 6.13):
            print('[ERROR]: Angle out of range!')
            return -1

        # Find the index of the joint
        joint_index = self.joint_states.name.index(joint)

        try:
            # Set the joint to the desired angle
            self.joint_states.position[joint_index] = angle

            time_interval = 0.1/100
            start_time = time.time()
            self.joint_states.position = [random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0), random.uniform(-2.99, 2.99), random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0), random.uniform(-6.0, 6.0)]
        
            # Iterate until 5 seconds have passed
            while time.time() - start_time < 5.0:
                # Set the joint positions to center

                # Update the timestamp
                now = time.time()
                self.joint_states.header.stamp.sec = int(now)
                self.joint_states.header.stamp.nanosec = int((now - int(now)) * 1e9)

                # Publish the joint states
                self.joint_state_publisher.publish(self.joint_states)

                # Sleep for the calculated time interval
                time.sleep(time_interval)

            print(f'Joint {joint} moved to {angle} radians')
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

    
def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
