#!/usr/bin/env python
import rospy
import numpy as np
# import math
import tf
# import tf2_ros
# from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
# from std_msgs.msg import Header, Float64MultiArray, Float64, Int8
# from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from sklearn.decomposition import PCA 
# import time
# from nav_msgs.msg import Path

class marker_to_target():
    def __init__(self):

        freq = 100
        self.x=0
        self.y=0
        self.z=0
        rospy.init_node('marker_to_target', anonymous=True)
        rate = rospy.Rate(freq)
        self.MarkerSub = rospy.Subscriber(
            "/sr/object_clusters_markerArray", MarkerArray, self.chatterCallback_Marker)    
        br_target = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        target_pose = Pose()
        self.data_received=False    
        print("Running...")
        while not rospy.is_shutdown():
            try:
                common_time = self.listener.getLatestCommonTime(
                'mocap_realsense_config_fixed', 'camera_depth_optical_frame') 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print(tf.LookupException())
                continue 
            br_target.sendTransform([self.x,self.y,self.z],[0,0,0,1],common_time,'target_position','camera_depth_optical_frame')
            rate.sleep()

    def chatterCallback_Marker(self, data):
        self.data = data.markers
        self.data_received=True
        x_vec=[]
        y_vec=[]
        z_vec=[]
        x_vec2=[]
        y_vec2=[]
        z_vec2=[]
        for i in range (len(self.data[0].points)):
            x_vec.append(self.data[0].points[i].x)
            y_vec.append(self.data[0].points[i].y)
            z_vec.append(self.data[0].points[i].z)
        for i in range (len(self.data[1].points)):
            x_vec2.append(self.data[1].points[i].x)
            y_vec2.append(self.data[1].points[i].y)
            z_vec2.append(self.data[1].points[i].z)
        x_vec_mean=np.mean(x_vec)
        y_vec_mean=np.mean(y_vec)
        z_vec_mean=np.mean(z_vec)
        x_vec2_mean=np.mean(x_vec2)
        y_vec2_mean=np.mean(y_vec2)
        z_vec2_mean=np.mean(z_vec2)
        zero_closest=np.linalg.norm([x_vec_mean,y_vec_mean,z_vec_mean])>np.linalg.norm([x_vec2_mean,y_vec2_mean,z_vec2_mean])
        self.x=np.mean(x_vec)*0.1+0.90*self.x
        self.y=np.mean(y_vec)*0.1+0.90*self.y
        self.z=np.mean(z_vec)*0.1+0.90*self.z
        # if zero_closest:
        #     self.x=np.mean(x_vec)*0.1+0.90*self.x
        #     self.y=np.mean(y_vec)*0.1+0.90*self.y
        #     self.z=np.mean(z_vec)*0.1+0.90*self.z
        # elif not zero_closest:
        #     self.x=np.mean(x_vec2)*0.1+0.90*self.x
        #     self.y=np.mean(y_vec2)*0.1+0.90*self.y
        #     self.z=np.mean(z_vec2)*0.1+0.90*self.z

        # if self.data

    
if __name__ == '__main__':
    marker_to_target()
