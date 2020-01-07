#!/usr/bin/env python
## Used to pick up objects from the top
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
import numpy as np
import math
import tf
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from std_msgs.msg import Header, Float64MultiArray, Float64, Int8, String
from sensor_msgs.msg import PointCloud
import time
from nav_msgs.msg import Path

class grasp_chesse():
    def __init__(self):

        freq = 200
        rospy.init_node('grasp', anonymous=True)

        self.init_broadcasts()
        self.init_topics()
        self.init_params()

        rate = rospy.Rate(freq)
        start = time.time()

    
        print("Starting loop")
        rot_mat=np.ones(4)
        while not rospy.is_shutdown():

            # Get End effector of robot
            try:
                trans_ee_real = self.listener.lookupTransform(
                    'world', 'iiwa_link_ee', rospy.Time(0))
                if not self.ee_svr_logged:
                    rospy.loginfo("ee_real transform received")
                    self.ee_svr_logged = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue            
            self.trans_ee_real=np.array(trans_ee_real[0])
            # Get target in robot frame
            try:
                # common_time = self.listener.getLatestCommonTime(
                # '/palm_link', '/target') 
                trans_world = self.listener.lookupTransform(
                    'mocap_world', 'world', rospy.Time(0))
                if not self.trans_world_logged:
                    rospy.loginfo("world transform received")
                    self.trans_world_logged = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            try:
                # common_time = self.listener.getLatestCommonTime(
                # '/palm_link', '/target') 
                trans_target_raw = self.listener.lookupTransform(
                    'mocap_world', 'mocap_test_object', rospy.Time(0))
                if not self.trans_target_logged:
                    rospy.loginfo("target transform received")
                    self.trans_target_logged = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue      
            if not self.target_read:
                trans_target=trans_target_raw
                trans_target_raw_rotated=tf.transformations.quaternion_multiply(trans_target_raw[1],trans_world[1])
                # rospy.loginfo("Target position and orientation locked")                
                # self.target_read=True
            # trans_target=trans_target_raw
            # trans_target_raw_rotated=tf.transformations.quaternion_multiply(trans_target_raw[1],trans_world[1])
            # print(tf.transformations.quaternion_matrix(trans_target[1]),np.append(np.array(trans_world[0]),1))

            ## Use the orientaion given by PCA
            offset=np.dot(tf.transformations.quaternion_matrix(trans_target[1]),np.array([-0.03,-0.04,0,1]))[:-1] #-0.03,-0.018 worked ok 20191022
            trans_world_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_world[0]),1))
            trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0])+offset,1))
            self.trans_target=trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0,0,0.14])+self.salad_offset

            # Testing grasping off chesse
            self.trans_target=np.array([0.65,-0.1,0.145])
            # print(self.salad_offset)

            ## Use a fixed orientation 170 degrees
            # transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_world[0]),1))
            # trans_world_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_world[0]),1))
            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0]),1))
            # self.trans_target=trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([-0.02,-0.005,0.19])

            ## Use a fixed orientation 180 degrees
            # trans_world_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_world[0]),1))
            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0]),1))
            # self.trans_target=trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0.00,0.015,0.18])            

            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0]),1))
            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0])+np.array([-0.005,-0.02,0.19]),1))
            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0])+np.array([-0.005,-0.02,0.0]),1))


            # try:
            #     # common_time = self.listener.getLatestCommonTime(
            #     # '/palm_link', '/target') 
            #     trans_pick = self.listener.lookupTransform(
            #         'world', 'mocap_test_object', rospy.Time(0))
            #     # if not self.:
            #     #     rospy.loginfo("world transform received")
            #     #     self.trans_world_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue      
            # print(trans_pick)       


            # self.trans_target=trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0,0,0.25])
            # self.trans_target=trans_world[0]
            self.attack_position=self.trans_target+np.array([-0.1,0,0.2])
            # self.attack_position=self.trans_target
            if not self.target_position_initialized:
                self.target_position=self.attack_position
                self.target_position_initialized=True
            

                
            if self.go_to_init_possition:
                # self.custom_command.data[0:3] = [0, np.deg2rad(90), 0]
                # self.custom_command.data[3:6] = [0, 0, 0.03]
                # t_start = time.time()
                # counter = 0
                # while counter < 50:
                #     # self.custom_command.data[0:3] = [0, (counter+1)/50*np.deg2rad(90), 0]
                #     self.RobotCommandPub.publish(self.custom_command)
                #     self.GrabPub.publish(0)
                #     counter = counter+1
                #     rate.sleep()
                self.go_to_init_possition = False
                raw_input('Waiting to start movement')
                self.go_to_init_possition2 = True
            if self.go_to_init_possition2:
                self.desired_orientation_cheese = [0, np.deg2rad(116), 0]
                self.custom_command.data[0:3] = self.desired_orientation_cheese
                self.custom_command.data[3:6] = [0, 0, 0.03]
                t_start = time.time()
                counter = 0
                while counter < 50:
                    self.GripperPub.publish(self.gripperMsg)
                    counter = counter+1
                    rate.sleep()
                self.go_to_init_possition2 = False
                raw_input('Waiting to start movement')               
            # self.desired_orientation=self.quat_to_direction(trans_target[1])
            self.desired_orientation=self.quat_to_direction(trans_target_raw_rotated)
            # self.desired_orientation= [0, np.deg2rad(180),0]  
            self.current_position = self.trans_ee_real
            direction_to_target = (self.target_position - self.current_position)
            distance_to_target = np.linalg.norm(direction_to_target)
            self.desired_velocity = self.robot_gain*(direction_to_target)
            desired_velocity_norm = np.linalg.norm(self.desired_velocity)
            if desired_velocity_norm>self.max_vel:
                self.desired_velocity=self.desired_velocity/desired_velocity_norm*self.max_vel
                
            # self.custom_command.data[0:3] = self.desired_orientation
            self.custom_command.data[0:3] = self.desired_orientation_cheese
            self.custom_command.data[3:6] = self.desired_velocity
            self.RobotCommandPub.publish(self.custom_command)
            # self.br_ee_target.sendTransform(trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([-0.025,0.009,0.0]), [0,0,0,1], rospy.Time.now(
            # ), 'target_position_rf', "world")
            self.br_ee_target.sendTransform(trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0,0,0.19]), trans_target_raw_rotated, rospy.Time.now(
            ), 'target_position_rf', "world")           
            self.br_ee_target_dbg.sendTransform(self.attack_position, [0,0,0,1], rospy.Time.now(
            ), 'attack_position_rf', "world")            
            rospy.loginfo_throttle(1, "Max speed: "+str(np.linalg.norm(self.desired_velocity)))
            # rospy.loginfo_throttle(1, "Target: "+str(self.target_position))

            # Go to initial position to grasp cheese
            if distance_to_target<0.025 and self.attack_reached==False:
                rospy.loginfo_throttle(1, ['Loop 1'])
                self.target_position=self.trans_target+np.array([-0.08,0,0.01])
                if self.counter0<100:
                    self.counter0+=1
                else:
                    self.attack_reached=True
                    self.max_vel=0.15
                    self.target_position=self.trans_target

            # Go to grasping position and grasp object
            elif distance_to_target<0.02 and self.attack_reached==True and self.target_reached==False:
                rospy.loginfo_throttle(1, ['Loop 2'])
                self.gripperMsg.rPR =self.gripper_closed
                self.target_position=self.trans_target
                if self.counter1 < 100:
                    self.GripperPub.publish(self.gripperMsg)
                    self.counter1 = self.counter1+1
                elif self.counter1>=100:
                    rospy.loginfo_throttle(1, ['Target reached'])
                    self.target_reached = True     
                print(self.counter1)

            # Go away from object
            if self.target_reached==True and self.pick_up_1==False:
                self.max_vel=0.4
                rospy.loginfo_throttle(1, ['Loop 3'])                
                self.target_position=self.trans_target+np.array([0,0,0.2])
                rospy.loginfo_throttle(1, ['Picked up object'])
                self.pick_up_1=True
            
            # Go to inital position to drop cheese
            elif self.pick_up_1==True and distance_to_target<0.05 and self.pick_up_2==False:
                rospy.loginfo_throttle(1, ['Loop 4'])                
                self.target_position=self.drop_position+np.array([0.05,0,0.2])
                self.pick_up_2=True

            # Go to initial position 2 to drop cheese
            elif self.pick_up_2==True and distance_to_target<0.02 and self.pick_up_3==False:
                rospy.loginfo_throttle(1, ['Loop 5'])                
                self.target_position=self.drop_position+np.array([0.05,0,0])
                self.pick_up_3=True    

            # Go to dropping position and drop cheese

            elif self.pick_up_3==True and distance_to_target<0.02 and self.pick_up_4==False:
                rospy.loginfo_throttle(1, ['Loop 6'])                    
                self.target_position=self.drop_position+np.array([-0.01,0,-0.02])
                self.gripperMsg.rPR =self.gripper_open
                if self.counter2 < 50:
                    self.GripperPub.publish(self.gripperMsg)
                    self.counter2 = self.counter2+1
                elif self.counter2>=50:
                    rospy.loginfo_throttle(1, ['Target reached'])
                self.pick_up_4=True
            
            elif self.pick_up_4==True and distance_to_target<0.021 and self.pick_up_5==False:
                rospy.loginfo_throttle(1, ['Loop 7']) 
                self.target_position=self.drop_position+np.array([-0.05,0,0.2])          
                self.desired_orientation_cheese = [0, np.deg2rad(140), 0]         
                self.pick_up_5=True

            # elif self.pick_up_5==True:
            #     raw_input('Repeat?')
            #     self.reset_bool()


            rospy.loginfo_throttle(1, [self.grasped==0])
            rospy.loginfo_throttle(1, ['Dist to target '+str(distance_to_target)])            
            rospy.loginfo_throttle(1, ['Target pos'+ str(self.target_position)])

            rate.sleep()


    def reset_bool(self):
        self.trans_target=np.array([0.65,-0.1,0.175])
        self.attack_reached=False
        self.target_reached=False
        self.allegroMsg = String()
        self.pick_up_1=False
        self.pick_up_2=False
        self.pick_up_3=False
        self.pick_up_4=False
        self.pick_up_5=False
        self.counter0=0
        self.counter1=0
        self.counter2=0
        self.gripper_closed=217
        self.gripper_open=190
        self.gripperMsg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripperMsg.rACT=1
        self.gripperMsg.rGTO=1
        self.gripperMsg.rATR=0
        self.gripperMsg.rPR =self.gripper_open
        self.gripperMsg.rSP =255
        self.gripperMsg.rFR =150

    def quat_to_direction(self, quat):
        R0 = tf.transformations.quaternion_matrix(quat)
        angle, direc, _ = tf.transformations.rotation_from_matrix(R0)
        angles = direc * angle  
        return angles

    def orientation_from_velocity(self, vec):
        axis_y = -np.array([0, 1, 0]) # Originaly negative
        axis_z = np.array(vec)
        axis_z_on_y = np.dot(axis_y, axis_z)
        axis_z = axis_z - axis_z_on_y * axis_y
        axis_z = axis_z/np.linalg.norm(axis_z)
        axis_x = np.cross(axis_y, axis_z)

        rot_mat = np.zeros((4, 4))
        rot_mat[:3, 0] = axis_x
        rot_mat[:3, 1] = axis_y
        rot_mat[:3, 2] = axis_z
        rot_mat[3, 3] = 1
        q_tf = tf.transformations.quaternion_from_matrix(rot_mat)
        return q_tf

    def quat_from_Vector(self, vec):
        # vec=np.array([1,0,0])
        axis_x = vec/np.linalg.norm(np.array(vec))
        axis_z = np.array([0,0,1])
        axis_z_on_x = np.dot(axis_z, axis_x)
        axis_z_2 = axis_z - axis_z_on_x * axis_x
        axis_z_3 = axis_z_2/np.linalg.norm(axis_z_2)
        axis_y = np.cross(axis_z_3, axis_x)
        # print(np.linalg.norm(axis_x), np.linalg.norm(axis_y), np.linalg.norm(axis_z))
        # print(np.dot(axis_x,axis_z))
        # axis_y = -np.array([0, 1, 0]) # Originaly negative
        # axis_z = np.array(vec)
        # axis_z_on_y = np.dot(axis_y, axis_z)
        # axis_z = axis_z - axis_z_on_y * axis_y
        # axis_z = axis_z/np.linalg.norm(axis_z)
        # axis_x = np.cross(axis_y, axis_z)

        rot_mat = np.zeros((4, 4))
        rot_mat[:3, 0] = axis_x
        rot_mat[:3, 1] = axis_y
        rot_mat[:3, 2] = axis_z_3
        rot_mat[3, 3] = 1
        q_tf = tf.transformations.quaternion_from_matrix(rot_mat)
        # print(rot_mat)
        return q_tf

    def get_svm_dir(self, curpos, cut, ok):
        svm_limit = 0.1
        offs = 0.001
        gain1 = 10
        gain2 = 10000
        next_svm_pos = self.desired_end_vec/100
        # self.get_robot_svr(next_svm_pos)
        if self.gamma_dist < cut-offs and ok:
            # self.svm_dir=-self.desired_end_vec*(cut-offs-self.gamma_dist)*(cut-offs-self.gamma_dist)*gain
            # next_svm_pos=(curpos-self.desired_end_vec)*(cut-offs-self.gamma_dist)*gain
            # next_svm_pos=(curpos-self.desired_end_vec)/10
            next_svm_pos = curpos+self.desired_end_vec/10
            # rospy.loginfo("SVR go back "+str(curpos)+" "+str(self.desired_end_vec))
            # next_svm_pos[1:]=0
            self.get_robot_svr(next_svm_pos)
            self.svm_dir=(self.world_to_svr_target-self.trans_ee_real)*(cut-offs-self.gamma_dist)*gain1
            # self.svm_dir = (self.world_to_svr_target-self.trans_ee_real) * \
            #     (cut-offs-self.gamma_dist)*(cut-offs-self.gamma_dist)*gain2

            self.svm_dir[1:] = 0

        elif self.gamma_dist >= cut+offs and ok:
            # self.svm_dir=self.desired_end_vec*(self.gamma_dist-cut-offs)*(self.gamma_dist-cut-offs)*gain
            # next_svm_pos=-(curpos-self.desired_end_vec)*(self.gamma_dist-cut-offs)*gain
            # next_svm_pos=-(curpos-self.desired_end_vec)/10
            next_svm_pos = curpos-self.desired_end_vec/10
            # rospy.loginfo("SVR go to "+str(curpos)+" "+str(self.desired_end_vec))
            # next_svm_pos[1:]=0
            self.get_robot_svr(next_svm_pos)
            self.svm_dir=(self.world_to_svr_target-self.trans_ee_real)*(self.gamma_dist-cut-offs)*gain1
            # self.svm_dir = (self.world_to_svr_target-self.trans_ee_real) * \
            #     (self.gamma_dist-cut-offs)*(self.gamma_dist-cut-offs)*gain2
            self.svm_dir[1:] = 0

        else:
            self.svm_dir = [0, 0, 0]
        # self.svm_dir=[0,0,0]
        # print(np.append(self.svm_dir,0))
        self.svm_dir_tmp = np.dot(tf.transformations.quaternion_matrix(
            self.trans_svr_ee[1]), np.append(self.svm_dir, 0))[:3]
        # print(self.svm_dir)
        # print(self.svm_dir_tmp)
        self.svm_dir_tmp[1:] = 0
        self.svm_dir2 = self.svm_dir_tmp * \
            self.quat_to_direction(
                tf.transformations.quaternion_inverse(self.trans_svr_ee[1]))
        svm_dir_norm = np.linalg.norm(self.svm_dir)
        # print(svm_dir_norm)
        if svm_dir_norm > svm_limit:
            self.svm_dir = self.svm_dir/svm_dir_norm*svm_limit

    def get_robot_svr(self, pos):
        # pos=self.trans_end_svr
        # self.svr_target=pos
        self.svr_target = pos
        # rospy.loginfo("SVR target"+str(self.svr_target))
        self.br_ee_svr_target.sendTransform(
            pos, [0, 0, 0, 1], rospy.Time.now(), 'svr_target', 'SVR')
        # self.listener.waitForTransform('/world', '/svr_target', rospy.Time.now(), rospy.Duration(4.0))
        transformed = False
        # print('svr')
        # while not transformed:
        try:
            trans_world_to_svr_target = np.array(self.listener.lookupTransform(
                'world', 'svr_target', rospy.Time(0)))
            self.world_to_svr_target = np.array(
                trans_world_to_svr_target[0])
            transformed = True
            if not self.ee_svr_target_logged2:
                rospy.loginfo("ee_svr_target transform received")
                self.ee_svr_target_logged2 = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            a = 1
            # continue

    def publish_svr_ee(self, trans_svr_ee):
        self.ee_in_svr.position.x = trans_svr_ee[0][0]
        self.ee_in_svr.position.y = trans_svr_ee[0][1]
        self.ee_in_svr.position.z = trans_svr_ee[0][2]
        self.ee_in_svr.orientation.x = trans_svr_ee[1][0]
        self.ee_in_svr.orientation.y = trans_svr_ee[1][1]
        self.ee_in_svr.orientation.z = trans_svr_ee[1][2]
        self.ee_in_svr.orientation.w = trans_svr_ee[1][3]
        self.RobotPosConvPub.publish(self.ee_in_svr)

    def chatterCallback_RobotEnd(self, data):
        self.end.position.x = data.position.x
        self.end.position.y = data.position.y
        self.end.position.z = data.position.z
        self.end.orientation.x = data.orientation.x
        self.end.orientation.y = data.orientation.y
        self.end.orientation.z = data.orientation.z
        self.end.orientation.w = data.orientation.w
        self.end_received = True

    def chatterCallback_desiredVel(self, data):
        self.desired_end_vec = np.array(
            [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])
        self.desired_end_received = True

    def chatterCallback_Gamma(self, data):
        self.gamma_dist = data.data

    def chatterCallback_Forces(self, data):
        self.forces_vec = 0.02*np.array(data.data)+0.98*self.forces_vec
        self.forces_vec[self.forces_vec < 0] = 0
        self.forces_received = True

    def publish_end_conv(self, data):
        self.Robot_conv.position.x = data[0][0]
        self.Robot_conv.position.y = data[0][1]
        self.Robot_conv.position.z = data[0][2]
        self.Robot_conv.orientation.x = data[1][0]
        self.Robot_conv.orientation.y = data[1][1]
        self.Robot_conv.orientation.z = data[1][2]
        self.Robot_conv.orientation.w = data[1][3]
        self.RobotPosConvPub.publish(self.Robot_conv)

    def publish_end_desired(self, data, data2):
        self.Robot_des.position.x = data[0]
        self.Robot_des.position.y = data[1]
        self.Robot_des.position.z = data[2]
        self.Robot_des.orientation.x = data2[0]
        self.Robot_des.orientation.y = data2[1]
        self.Robot_des.orientation.z = data2[2]
        self.Robot_des.orientation.w = data2[3]
        self.RobotPosDesiredConvertedPub.publish(self.Robot_des)

    def chatterCallback_Grasped(self, data):
        self.grasped = data.data

    def chatterCallback_Salad(self, data):
        self.salad = data.data
        if self.salad==1:
            self.salad_offset=[0,0,-0.04]

        
    # rotate vector v1 by quaternion q1
    def qv_mult(self, q1, v1):
        v_norm = np.linalg.norm(v1)
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]*v_norm

    def pubish_on_point_cloud(self, X):
        pointCloud = PointCloud()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'SVR'
        pointCloud.header = header
        pointCloud.points = []
        for i in range(len(X)):
            Point_temp = Point32()
            Point_temp.x = X[i][0]
            Point_temp.y = X[i][1]
            Point_temp.z = X[i][2]

            pointCloud.points.append(Point_temp)
        self.CloudPub.publish(pointCloud)

    def load_pointcloud(self):
        self.pointcloud = np.loadtxt(
            "/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/Pointcloud/pointcloud3.txt")

    def publish_arm_and_shoulder(self):
        try:
            # trans_end_hand = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/mocap_hand_filtered', rospy.Time(0))
            trans_end_hand = self.listener.lookupTransform(
                '/SVR', '/mocap_hand_filtered', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to Hand")
        self.Robot_hand.pose.position.x = trans_end_hand[0][0]
        self.Robot_hand.pose.position.y = trans_end_hand[0][1]
        self.Robot_hand.pose.position.z = trans_end_hand[0][2]
        self.Robot_hand.pose.orientation.x = trans_end_hand[1][0]
        self.Robot_hand.pose.orientation.y = trans_end_hand[1][1]
        self.Robot_hand.pose.orientation.z = trans_end_hand[1][2]
        self.Robot_hand.pose.orientation.w = trans_end_hand[1][3]
        self.RobotHandPub.publish(self.Robot_hand)

        try:
            # trans_end_shoulder = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/SVR', rospy.Time(0))
            trans_end_shoulder = self.listener.lookupTransform(
                '/SVR', '/SVR', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to SVR")
        self.Robot_shoulder.pose.position.x = trans_end_shoulder[0][0]
        self.Robot_shoulder.pose.position.y = trans_end_shoulder[0][1]
        self.Robot_shoulder.pose.position.z = trans_end_shoulder[0][2]
        self.Robot_shoulder.pose.orientation.x = trans_end_shoulder[1][0]
        self.Robot_shoulder.pose.orientation.y = trans_end_shoulder[1][1]
        self.Robot_shoulder.pose.orientation.z = trans_end_shoulder[1][2]
        self.Robot_shoulder.pose.orientation.w = trans_end_shoulder[1][3]
        self.RobotShoulderPub.publish(self.Robot_shoulder)

    def init_broadcasts(self):
        self.listener = tf.TransformListener()
        self.br_robot_base_fixed = tf.TransformBroadcaster()
        self.br_ee_target = tf.TransformBroadcaster()
        self.br_ee_target_dbg = tf.TransformBroadcaster()

    def init_topics(self):
        self.RobotCommandPub = rospy.Publisher(
            "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)        
        self.GrabPub = rospy.Publisher(
             "/grab", Int8, queue_size=3)
        self.GraspedSub = rospy.Subscriber(
            "/grasped", Int8, self.chatterCallback_Grasped)
        self.SaladSub = rospy.Subscriber(
            "/salad", Int8, self.chatterCallback_Salad)
        self.AllegroPub = rospy.Publisher(
            "/allegroHand_1/lib_cmd", String, queue_size=3)
        self.GripperPub = rospy.Publisher(
            "/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=3
        )


    def init_params(self):
        # Initial position
        self.go_to_init_possition = True
        self.go_to_init_possition2 = False
        self.custom_command = Float64MultiArray()
        self.custom_command.data = [0, 0, 0, 0, 0, 0]
        self.salad_offset = np.zeros(3)

        # Target
        self.target_reached = False
        self.target_reached_init = False
        self.attack_reached = False
        self.target_position_initialized=False
        self.target_read = False

        # End effector transform
        self.ee_svr_logged = False
        self.trans_target_logged = False
        self.trans_world_logged = False

        # Robot parameters
        self.max_vel = 0.4
        self.robot_gain = 6

        # Grasping
        self.grasped = 0
        self.allegroMsg = String()
        self.drop_position = np.array([0.70,0.1,0.25])
        self.pick_up_1=False
        self.pick_up_2=False
        self.pick_up_3=False
        self.pick_up_4=False
        self.pick_up_5=False
        self.counter0=0
        self.counter1=0
        self.counter2=0
        self.gripper_closed=217
        self.gripper_open=150
        self.gripperMsg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripperMsg.rACT=1
        self.gripperMsg.rGTO=1
        self.gripperMsg.rATR=0
        self.gripperMsg.rPR =self.gripper_open
        self.gripperMsg.rSP =255
        self.gripperMsg.rFR =150
        



    def publish_path(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1
        pose.pose.position.y = 1
        pose.pose.position.z = 1
        self.path.poses.append(pose)
        self.path.header.frame_id = 'mocap world'
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    grasp_chesse()
