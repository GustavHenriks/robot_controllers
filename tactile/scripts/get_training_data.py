#!/usr/bin/env python
import rospy
import numpy as np
# import math
import tf.transformations
import tf
# import tf2_ros
# from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
from std_msgs.msg import Int8, Header
# from sensor_msgs.msg import PointCloud
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose, Point32
from visualization_msgs.msg import MarkerArray
from sklearn.decomposition import PCA 
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud
import time
# import time
# from nav_msgs.msg import Path

class get_training_data():
    def __init__(self):
        rospy.init_node('get_training_data')
        freq = 200
        rate = rospy.Rate(freq)
        self.listener = tf.TransformListener()
        self.ee_svr_logged = False
        self.robot_vec=[]
        self.timestr = time.strftime("%Y%m%d%H%M%S")

        print("Running...")
        while not rospy.is_shutdown():
            try:            
                self.trans_ee_test = self.listener.lookupTransform(
                    'world','iiwa_link_ee', rospy.Time(0))
                if not self.ee_svr_logged:
                    rospy.loginfo("ee_real transform received")
                    self.ee_svr_logged = True
                    self.trans_ee_test_origin=self.trans_ee_test
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  
            self.robot_vec.append(self.trans_ee_test[0])
            if np.linalg.norm(np.array(self.trans_ee_test_origin[0])-np.array(self.trans_ee_test[0]))>0.01:
                self.save_trajectory()
                print('recording')
            rate.sleep()

    def save_trajectory(self):
        with open("recordings/"+self.timestr+"_robot_trajectory.txt", mode='w') as f:  # I add the mode='w'
            for i in range(len(self.robot_vec)-1):
                f.write("%f,"%float(self.robot_vec[i][0]))
                f.write("%f,"%float(self.robot_vec[i][1]))
                f.write("%f,\n"%float(self.robot_vec[i][2]))


    
    
if __name__ == '__main__':
    get_training_data()
