#!/usr/bin/env python
import rospy
import numpy as np
# import math
# import tf
# from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import PointCloud
# import time


class send_command():
    def __init__(self):
        freq = 200
        rospy.init_node('send_command', anonymous=True)
        rate = rospy.Rate(freq)
        self.RobotCommandPub = rospy.Publisher(
            "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)
        self.RobotCommandBuffer = rospy.Subscriber(
            "/Robot/command_buffer", Float64MultiArray, self.chatterCallback_buffer)
        self.custom_command = Float64MultiArray()
        self.custom_command.data = [0, 0, 0, 0, 0, 0]
        self.data_received=False
        print("Running...")
        while not rospy.is_shutdown():
            if self.data_received==True:
                self.RobotCommandPub.publish(self.custom_command)
            rate.sleep()

    def chatterCallback_buffer(self, data):
        self.custom_command.data=data.data
        self.data_received=True

if __name__ == '__main__':
    send_command()
