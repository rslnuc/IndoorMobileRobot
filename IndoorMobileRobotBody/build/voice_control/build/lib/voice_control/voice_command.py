from flask import Flask, request

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import Requests

import time
import numpy as np
from . import RoverDescription as Rover

class get_move_cmds(Node):

    def __init__(self):
        super().__init__('rover_state_controler')
        
        # Create the publishers to the roboteq motors.
        timer_period = 0.1  # seconds
        self.pub_du = self.create_publisher(Float32MultiArray, 'rov/motors', 5)
        self.timer = self.create_timer(timer_period, self.drive_unit_callback)
        self.i = 0
        
        # RPM Vector to publish to the roboteq controller.
        self.vel_vector = [0, 0, 0, 0]
        
        # URL for alexa to publish to.
        self.url = pass
        
        # State
        self.state = Requests.get(self.url)

    # Computes the velocity vector based on the controller input.
    def receive_voice_command(self, msg):
        if(msg.string.lower() == 'forward'):
            self.vel_vector = [100, 100, 100, 100]
        elif(msg.string.lower() == 'backward'):
            self.vel_vector = [-100, -100, -100, -100]
        elif(msg.string.lower() == 'right'):
            self.vel_vector = [-100, 100, 100, -100]
        elif(msg.string.lower() == 'left'):
            self.vel_vector = [100, -100, -100, 100]
        else:
            self.vel_vector = [0, 0, 0, 0]

    # Publishes to the roboteq controller node.
    def drive_unit_callback(self):
        msg = Float32MultiArray()
        # print(self.rpm_vec)
        msg.data = self.rpm_vec
        # print(msg.data)

        # Publish to all the motor drivers.
        self.pub_du.publish(msg)
        self.i += 1 

def main(args=None):
    rclpy.init(args=args)

    sub_move_cmds = get_move_cmds()

    rclpy.spin(sub_move_cmds)

    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
