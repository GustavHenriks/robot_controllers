#!/usr/bin/env python
import rospy
import numpy as np
# import math
import tf.transformations
import tf
# import tf2_ros
# from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
# from std_msgs.msg import Header, Float64MultiArray, Float64, Int8
# from sensor_msgs.msg import PointCloud
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose
from visualization_msgs.msg import MarkerArray
from sklearn.decomposition import PCA 
import matplotlib.pyplot as plt
# import time
# from nav_msgs.msg import Path

class marker_to_target():
    def __init__(self):

        freq = 10
        self.x=0
        self.y=0
        self.z=0
        self.x_dir=np.zeros(3)
        self.x_pca_axis_old=np.array([0,1,0])
        self.counter=0
        rospy.init_node('marker_to_target', anonymous=True)
        rate = rospy.Rate(freq)
        self.MarkerSub = rospy.Subscriber(
            "/sr/object_clusters_markerArray", MarkerArray, self.chatterCallback_Marker)  
        br_target = tf.TransformBroadcaster()
        br_test = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        target_pose = Pose()
        self.data_received=False    
        z_in_world=np.array([0,0,1,1])

        # self.plot_test()
        print("Running...")
        while not rospy.is_shutdown():
            try:            
                trans_ee_real = self.listener.lookupTransform(
                    'camera_depth_optical_frame', 'mocap_world', rospy.Time(0))
                # if not self.ee_svr_logged:
                #     rospy.loginfo("ee_real transform received")
                #     self.ee_svr_logged = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue            
            # print(trans_ee_real)

            try:
                common_time = self.listener.getLatestCommonTime(
                'mocap_realsense_config_fixed', 'camera_depth_optical_frame') 
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print(tf.LookupException())
                continue
            self.up=np.dot(tf.transformations.quaternion_matrix(trans_ee_real[1]),z_in_world)[:-1]
            quat=self.quat_from_Vector_y(self.x_dir)
            # quat=self.quat_from_Vector(np.append(self.x_dir,0))
            # print(np.linalg.norm(quat))
            br_target.sendTransform([self.x,self.y,self.z],quat,common_time,'target_position','camera_depth_optical_frame')
            # br_test.sendTransform([self.x+0.1,self.y,self.z],,common_time,'target_position','camera_depth_optical_frame')
            # br_target.sendTransform([self.x,self.y,self.z],trans_ee_real[1],common_time,'target_position','camera_depth_optical_frame')
            # br_test.sendTransform([self.x,self.y,self.z],trans_ee_real[1],common_time,'test','camera_depth_optical_frame')
            # a1,a2,a3=tf.transformations.rotation_from_matrix(tf.transformations.quaternion_matrix(trans_ee_real[1]))
            rate.sleep()

    def chatterCallback_Marker(self, data):
        self.data = data.markers
        self.data_received=True
        x_vec=[]
        y_vec=[]
        z_vec=[]
        # x_vec2=[]
        # y_vec2=[]
        # z_vec2=[]
        for i in range (len(self.data[0].points)):
            x_vec.append(self.data[0].points[i].x)
            y_vec.append(self.data[0].points[i].y)
            z_vec.append(self.data[0].points[i].z)
        # for i in range (len(self.data[1].points)):
        #     x_vec2.append(self.data[1].points[i].x)
        #     y_vec2.append(self.data[1].points[i].y)
        #     z_vec2.append(self.data[1].points[i].z)
        x_vec_mean=np.mean(x_vec)
        y_vec_mean=np.mean(y_vec)
        z_vec_mean=np.mean(z_vec)
        # x_vec2_mean=np.mean(x_vec2)
        # y_vec2_mean=np.mean(y_vec2)
        # z_vec2_mean=np.mean(z_vec2)
        self.x=np.mean(x_vec)*0.1+0.90*self.x
        self.y=np.mean(y_vec)*0.1+0.90*self.y
        self.z=np.mean(z_vec)*0.1+0.90*self.z
        # zero_closest=np.linalg.norm([x_vec_mean,y_vec_mean,z_vec_mean])>np.linalg.norm([x_vec2_mean,y_vec2_mean,z_vec2_mean])
        # if zero_closest:
        #     self.x=np.mean(x_vec)*0.1+0.90*self.x
        #     self.y=np.mean(y_vec)*0.1+0.90*self.y
        #     self.z=np.mean(z_vec)*0.1+0.90*self.z
        # elif not zero_closest:
        #     self.x=np.mean(x_vec2)*0.1+0.90*self.x
        #     self.y=np.mean(y_vec2)*0.1+0.90*self.y
        #     self.z=np.mean(z_vec2)*0.1+0.90*self.z
        # self.x=np.mean(x_vec)*0.1+0.90*self.x
        # self.y=np.mean(y_vec)*0.1+0.90*self.y
        # self.z=np.mean(z_vec)*0.1+0.90*self.z

        pca = PCA(n_components=3,whiten=True)
        # # print(np.stack((x_vec,y_vec,z_vec),axis=-1))
        print(np.mean(z_vec))
        x_vec=x_vec-x_vec_mean
        y_vec=y_vec-y_vec_mean
        z_vec=z_vec-z_vec_mean
        print(np.mean(z_vec))
        pca.fit(np.stack((x_vec,y_vec,z_vec),axis=-1))
        # pca.fit(np.stack((x_vec,y_vec),axis=-1))
        # print(pca.components_.T)
        print("Var: ", pca.explained_variance_ratio_)
        V = pca.components_
        # V /= V.std()
        # # print("V=",V.T)
        # # print(V.T[0,:],V.T[1,:],V.T[2,:])
        # # x_pca_axis = V.T[0,:]
        # # y_pca_axis = V.T[1,:]
        # # z_pca_axis = V.T[2,:]
        x_pca_axis, y_pca_axis, z_pca_axis = V
        # x_pca_axis, y_pca_axis= V.T
        # if np.argmax(x_pca_axis)!=np.argmax(np.abs(x_pca_axis)):
        if x_pca_axis[0] < 0:
            x_pca_axis=-x_pca_axis        
        # if z_pca_axis[2] < 0:
        #     x_pca_axis=-x_pca_axis
        #     y_pca_axis=-y_pca_axis
        #     z_pca_axis=-x_pca_axis
            
        print( x_pca_axis)
        # print(np.dot(self.x_pca_axis_old,x_pca_axis))
        # if self.counter>50 and np.dot(self.x_pca_axis_old,x_pca_axis)<0.5:
        #     x_pca_axis=self.x_pca_axis_oldf
        # if np.dot(self.x_pca_axis_best,x_pca)
        # if np.sign(max(x_pca_axis))==-1:
        #     x_pca_axis=-x_pca_axis
        self.x_dir=x_pca_axis*0.01+0.99*self.x_dir
        # self.x_dir=x_pca_axis
        self.x_pca_axis_old=x_pca_axis
        # self.counter=self.counter+1
        # self.x_dir=[0,1,0]
        # print("X_v: ", x_pca_axis)
        # # print(x_pca_axis[0])
        # # plt.plot(x_vec_mean,y_vec_mean,z_vec_mean,1,1,1, color='red')
        # # plt.quiver(x_vec_mean,y_vec_mean,z_vec_mean,x_pca_axis,y_pca_axis,z_pca_axis)
        # # plt.quiver(x_vec_mean,y_vec_mean,z_vec_mean,x_pca_axis,y_pca_axis,z_pca_axis)
        # V = np.array([[1,1,-1],[-2,2,1],[4,-7,1]])
        # origin = [0], [0] # origin point

        # plt.quiver(0,0,0, V[:,0], V[:,1], V[:,2], color=['r','b','g'], scale=21)
        # plt.show()
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # print(np.linalg.norm([x_pca_axis,y_pca_axis,z_pca_axis],axis=1))
        # ax.quiver([x_vec_mean,x_vec_mean,x_vec_mean], [y_vec_mean,y_vec_mean,y_vec_mean], [z_vec_mean,z_vec_mean,z_vec_mean], x_pca_axis, y_pca_axis, z_pca_axis,length=0.01)
        # fig = plt.figure(1, figsize=(4, 3))
        # elev = -40
        # azim = -80
        # ax = Axes3D(fig, rect=[0, 0, .95, 1], elev=elev, azim=azim)
        # ax = fig.gca(projection='3d')
        # # x_pca_plane = np.r_[x_pca_axis[:2], - x_pca_axis[1::-1]]
        # # y_pca_plane = np.r_[y_pca_axis[:2], - y_pca_axis[1::-1]]
        # # z_pca_plane = np.r_[z_pca_axis[:2], - z_pca_axis[1::-1]]
        # # x_pca_plane.shape = (2, 2)
        # # y_pca_plane.shape = (2, 2)
        # # z_pca_plane.shape = (2, 2)
        # # ax.plot_surface(x_pca_plane, y_pca_plane, z_pca_plane)
        # # ax.w_xaxis.set_ticklabels([])
        # # ax.w_yaxis.set_ticklabels([])
        # # ax.w_zaxis.set_ticklabels([])        
        # ax.scatter(x_vec, y_vec, z_vec, marker='+', alpha=.4)
        # ax.quiver(0,0,0, x_pca_axis[0],x_pca_axis[1],x_pca_axis[2],length=0.01,color='red')
        # ax.quiver(0,0,0, y_pca_axis[0],y_pca_axis[1],y_pca_axis[2],length=0.01,color='green')
        # ax.quiver(0,0,0, z_pca_axis[0],z_pca_axis[1],z_pca_axis[2],length=0.01,color='blue')
        # plt.show()

    def quat_from_Vector(self, vec):
        # vec=np.array([1,0,0])
        ## Align for x
        # axis_x = vec/np.linalg.norm(np.array(vec))
        # axis_z = -np.array([0,0,1])
        # axis_z_on_x = np.dot(axis_z, axis_x)
        # axis_z_2 = axis_z - axis_z_on_x * axis_x
        # axis_z_3 = axis_z_2/np.linalg.norm(axis_z_2)
        # axis_y = np.cross(axis_z_3, axis_x)
        ## Align for z
        axis_x = vec/np.linalg.norm(np.array(vec))
        axis_z = -self.up/np.linalg.norm(self.up)
        axis_x_on_z = np.dot(axis_x, axis_z)
        axis_x_2 = axis_x - axis_x_on_z * axis_z
        axis_x_3 = axis_x_2/np.linalg.norm(axis_x_2)
        axis_y = np.cross(axis_z, axis_x_3)
        axis_y_2 = axis_y/np.linalg.norm(axis_y)
        # print(np.linalg.norm(axis_x), np.linalg.norm(axis_y), np.linalg.norm(axis_z))
        # print(np.dot(axis_x,axis_z))
        # axis_y = -np.array([0, 1, 0]) # Originaly negative
        # axis_z = np.array(vec)
        # axis_z_on_y = np.dot(axis_y, axis_z)
        # axis_z = axis_z - axis_z_on_y * axis_y
        # axis_z = axis_z/np.linalg.norm(axis_z)
        # axis_x = np.cross(axis_y, axis_z)
        # print(np.linalg.norm(axis_x_3),np.linalg.norm(axis_y_2),np.linalg.norm(axis_z))
        rot_mat = np.zeros((4, 4))
        rot_mat[:3, 0] = axis_x_3
        rot_mat[:3, 1] = axis_y_2
        rot_mat[:3, 2] = axis_z
        rot_mat[3, 3] = 1
        q_tf = tf.transformations.quaternion_from_matrix(rot_mat)
        # print(rot_mat)
        # return q_tf/np.linalg.norm(q_tf)
        return q_tf

    def quat_from_Vector_y(self, vec):
        # vec=np.array([1,0,0])
        ## Align for x
        # axis_x = vec/np.linalg.norm(np.array(vec))
        # axis_z = -np.array([0,0,1])
        # axis_z_on_x = np.dot(axis_z, axis_x)
        # axis_z_2 = axis_z - axis_z_on_x * axis_x
        # axis_z_3 = axis_z_2/np.linalg.norm(axis_z_2)
        # axis_y = np.cross(axis_z_3, axis_x)
        ## Align for z
        axis_x = vec/np.linalg.norm(np.array(vec))
        axis_z = -self.up/np.linalg.norm(self.up)
        axis_x_on_z = np.dot(axis_x, axis_z)
        axis_x_2 = axis_x - axis_x_on_z * axis_z
        axis_x_3 = axis_x_2/np.linalg.norm(axis_x_2)
        axis_y = np.cross(axis_z, axis_x_3)
        axis_y_2 = axis_y/np.linalg.norm(axis_y)
        # print(np.linalg.norm(axis_x), np.linalg.norm(axis_y), np.linalg.norm(axis_z))
        # print(np.dot(axis_x,axis_z))
        # axis_y = -np.array([0, 1, 0]) # Originaly negative
        # axis_z = np.array(vec)
        # axis_z_on_y = np.dot(axis_y, axis_z)
        # axis_z = axis_z - axis_z_on_y * axis_y
        # axis_z = axis_z/np.linalg.norm(axis_z)
        # axis_x = np.cross(axis_y, axis_z)
        # print(np.linalg.norm(axis_x_3),np.linalg.norm(axis_y_2),np.linalg.norm(axis_z))
        rot_mat = np.zeros((4, 4))
        rot_mat[:3, 0] = axis_y_2
        rot_mat[:3, 1] = -axis_x_3
        rot_mat[:3, 2] = axis_z
        rot_mat[3, 3] = 1
        q_tf = tf.transformations.quaternion_from_matrix(rot_mat)
        # print(rot_mat)
        # return q_tf/np.linalg.norm(q_tf)
        return q_tf    


    def plot_test(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Make the grid
        x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),
                            np.arange(-0.8, 1, 0.2),
                            np.arange(-0.8, 1, 0.8))

        # Make the direction data for the arrows
        u = np.sin(np.pi * x) * np.cos(np.pi * y) * np.cos(np.pi * z)
        v = -np.cos(np.pi * x) * np.sin(np.pi * y) * np.cos(np.pi * z)
        w = (np.sqrt(2.0 / 3.0) * np.cos(np.pi * x) * np.cos(np.pi * y) *
            np.sin(np.pi * z))

        # ax.quiver(x, y, z, u, v, w, length=0.1)
        ax.quiver(0,0,0,-1,0,0,length=0.1)

        plt.show()

    
    
if __name__ == '__main__':
    marker_to_target()
