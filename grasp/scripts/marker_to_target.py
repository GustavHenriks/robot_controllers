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
            # self.plot_pcl()
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
        with open("points2.txt", mode='w') as f:  # I add the mode='w'
            for i in range(len(x_vec)):
                f.write("%f,"%float(x_vec[i]))
                f.write("%f,"%float(y_vec[i]))
                f.write("%f,\n"%float(z_vec[i]))

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

    def plot_pcl(self):
        Vec=[0.012795,0.025430,-0.002553,
            0.007500,0.025491,-0.003092,
            0.000737,0.024792,-0.003005,
            -0.006527,0.024597,-0.002464,
            -0.011672,0.023658,-0.002347,
            0.018285,0.016641,-0.003347,
            0.013872,0.019465,-0.003968,
            0.007149,0.019629,-0.005722,
            0.000377,0.019634,-0.005694,
            -0.006845,0.019610,-0.004312,
            -0.018194,0.017897,-0.002347,
            0.020356,0.012656,-0.003162,
            0.013892,0.012540,-0.005639,
            0.007139,0.012795,-0.007382,
            0.000222,0.012783,-0.007728,
            -0.006694,0.012504,-0.006357,
            -0.013930,0.012655,-0.004618,
            -0.019083,0.012570,-0.002942,
            0.024851,0.002353,-0.002347,
            0.020376,0.005336,-0.003165,
            0.013908,0.005689,-0.005877,
            0.006931,0.005632,-0.007940,
            0.000378,0.005695,-0.007772,
            -0.006781,0.005586,-0.006658,
            -0.014026,0.005725,-0.004253,
            -0.018027,0.006354,-0.002490,
            0.025155,-0.000158,-0.002347,
            0.020950,-0.001416,-0.003567,
            0.014179,-0.001342,-0.005712,
            0.007084,-0.001181,-0.006816,
            0.000328,-0.001212,-0.007087,
            -0.006841,-0.001460,-0.006551,
            -0.013497,-0.001066,-0.003907,
            -0.017759,0.000831,-0.002347,
            0.019514,-0.006487,-0.002781,
            0.013717,-0.007906,-0.003938,
            0.007178,-0.008171,-0.005235,
            0.000407,-0.008142,-0.004891,
            -0.006348,-0.008045,-0.003651,
            -0.011998,-0.006146,-0.002561,
            0.011155,-0.012104,-0.002347,
            0.006789,-0.013239,-0.002597,
            0.000899,-0.013256,-0.002494,
            -0.004190,-0.012358,-0.002347,
            0.005766,0.032120,0.001891,
            0.002181,0.031508,0.001403,
            0.014536,0.026267,-0.001347,
            0.007287,0.029056,-0.000966,
            0.000038,0.028074,-0.001124,
            -0.005874,0.026675,-0.001347,
            -0.011872,0.024760,-0.000597,
            0.016572,0.023086,-0.001347,
            -0.019670,0.017827,-0.001193,
            0.023443,0.009344,-0.001347,
            -0.021213,0.012441,-0.000619,
            0.025337,0.007254,-0.001124,
            0.023539,0.006862,-0.001347,
            0.025734,-0.003407,-0.001347,
            0.024207,-0.004017,-0.001347,
            -0.016329,-0.003419,-0.001256,
            -0.021062,-0.001419,0.000937,
            0.026984,-0.008801,0.001173,
            0.021928,-0.008978,-0.000313,
            0.015951,-0.010749,-0.001180,
            -0.009386,-0.010507,-0.001347,
            -0.014487,-0.008688,0.000296,
            -0.019012,-0.007235,0.002686,
            0.025779,-0.013446,0.003653,
            0.020906,-0.014802,0.002740,
            0.014497,-0.015312,0.000768,
            0.007804,-0.016321,-0.000796,
            -0.000251,-0.016408,-0.000838,
            -0.007136,-0.015455,0.000296,
            -0.013497,-0.014232,0.002309,
            -0.017912,-0.013235,0.003653,
            0.012855,-0.019776,0.002961,
            0.000472,-0.019940,0.002005,
            -0.004872,-0.019610,0.003453,
            0.005043,0.034350,0.004653,
            -0.024169,-0.004600,0.004653,
            0.030361,-0.011352,0.004653,
            -0.021968,-0.008839,0.005732,
            -0.027533,-0.008295,0.008182,
            -0.031528,-0.008488,0.009653,
            0.027135,-0.014927,0.004690,
            0.022896,-0.017973,0.004653,
            -0.009758,-0.018622,0.004653,
            -0.014873,-0.017479,0.006067,
            -0.020910,-0.015417,0.008146,
            -0.026386,-0.014969,0.010327,
            -0.006966,-0.021504,0.007813,
            -0.013485,-0.020361,0.009247,
            -0.018403,-0.019461,0.010453,
            -0.029655,-0.015142,0.011653,
            -0.031666,-0.013908,0.011653,
            -0.007443,-0.024627,0.012153,
            -0.014008,-0.023746,0.012775,
            -0.020633,-0.021880,0.012735,
            -0.012999,-0.027132,0.014353]
        # plt.plot(Vec[:3:],Vec[1:3:],Vec[2:3:])
        fig = plt.figure()
        ax = Axes3D(fig, rect=[0, 0, .95, 1], elev=-40, azim=-80)
        ax = fig.gca(projection='3d')
        # x_pca_plane = np.r_[x_pca_axis[:2], - x_pca_axis[1::-1]]
        # y_pca_plane = np.r_[y_pca_axis[:2], - y_pca_axis[1::-1]]
        # z_pca_plane = np.r_[z_pca_axis[:2], - z_pca_axis[1::-1]]
        # x_pca_plane.shape = (2, 2)
        # y_pca_plane.shape = (2, 2)
        # z_pca_plane.shape = (2, 2)
        # ax.plot_surface(x_pca_plane, y_pca_plane, z_pca_plane)
        # ax.w_xaxis.set_ticklabels([])
        # ax.w_yaxis.set_ticklabels([])
        # ax.w_zaxis.set_ticklabels([])        
        ax.scatter(Vec[::3],Vec[1::3],Vec[2::3], marker='+', alpha=.4)
        plt.show()

    
    
if __name__ == '__main__':
    marker_to_target()
