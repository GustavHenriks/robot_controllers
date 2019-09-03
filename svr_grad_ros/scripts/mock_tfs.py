#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Float64MultiArray


class mock_tfs():
    def __init__(self):

        freq = 200
        rospy.init_node('mock_tfs', anonymous=True)
        self.init_tf_values()
        br_h = tf.TransformBroadcaster()
        br_s = tf.TransformBroadcaster()
        br_b = tf.TransformBroadcaster()
        rate=rospy.Rate(freq)
        print('Running...')
        while not rospy.is_shutdown():
            br_h.sendTransform(self.hand_pos, self.hand_ori, rospy.Time.now(),
                        '/mocap_hand', "mocap_world")
            br_s.sendTransform(self.shoulder_pos, self.shoulder_ori, rospy.Time.now(),
                        '/mocap_shoulder', "mocap_world")
            br_b.sendTransform(self.base_pos, self.base_ori, rospy.Time.now(),
                        '/mocap_robot_base', "mocap_world")                                    
            rate.sleep()
       
    def init_tf_values(self):
        self.hand_pos=[-1.49783432484,-0.571042716503,1.19645535946]
        self.hand_ori=[-0.014699744992,0.00233529345132,-0.010908938013,-0.999829769135]
        self.shoulder_pos=[-0.88842022419,-0.599843502045,1.2341606617]
        self.shoulder_ori=[-0.00814420729876,0.00552493380383,-0.00190541322809,-0.999949753284]
        self.base_pos=[-1.05536401272,0.263663589954,0.820718407631]
        self.base_ori=[-0.000998265575618,0.000763374671806,-0.000574508041609,0.99999910593]


if __name__ == '__main__':
    mock_tfs()
