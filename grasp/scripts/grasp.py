#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped, TwistStamped
from std_msgs.msg import Header, Float64MultiArray, Float64, Int8
from sensor_msgs.msg import PointCloud
import time
from nav_msgs.msg import Path

class convert_tf():
    def __init__(self):

        freq = 200
        rospy.init_node('grasp', anonymous=True)

        # self.listener = tf.TransformListener()
        # br_svr = tf.TransformBroadcaster()
        # br_ee = tf.TransformBroadcaster()
        # br_ee_conv = tf.TransformBroadcaster()
        # br_robot_base_fixed = tf.TransformBroadcaster()
        # self.br_svr_rotated = tf.TransformBroadcaster()
        # br_hand_filter = tf.TransformBroadcaster()
        # br_ee_svr = tf.TransformBroadcaster()
        # br_ee_target = tf.TransformBroadcaster()
        # br_ee_finger = tf.TransformBroadcaster()
        # self.br_ee_line_target = tf.TransformBroadcaster()
        # self.br_ee_line_target2 = tf.TransformBroadcaster()
        # self.br_ee_svr_target = tf.TransformBroadcaster()
        # br_ee_debug_svr = tf.TransformBroadcaster()
        # br_ee_debug_com = tf.TransformBroadcaster()
        # br_ee_debug_quat_svr = tf.TransformBroadcaster()
        # br_ee_debug_quat_com = tf.TransformBroadcaster()
        # br_palm_link = tf.TransformBroadcaster()

        
        # br_robot_base_world_mocap = tf.TransformBroadcaster()
        self.init_broadcasts()
        self.init_topics()
        self.init_params()
        # self.load_pointcloud()

        rate = rospy.Rate(freq)
        start = time.time()

        # calibration of the arm orientation
        # print('Calibrating the arm orientation')
        # delay = 1
        # displacement = np.array([0,0,0,0])
        # while time.time() < start+delay and not rospy.is_shutdown():
        #     try:
        #         trans_arm = self.listener.lookupTransform(
        #             '/mocap_hand', '/mocap_shoulder', rospy.Time(0))
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         continue
        #     self.trans_arm = trans_arm
        #     self.p_arm = np.array(
        #         [trans_arm[0][0], trans_arm[0][1], trans_arm[0][2]])

        #     self.p_arm = self.p_arm/1.7

        #     br_svr.sendTransform(self.p_arm, trans_arm[1], rospy.Time.now(),
        #                             'mocap_svr', "mocap_hand")

            # freezing the robot_base
            # try:
            #     trans_world_base = self.listener.lookupTransform(
            #         '/mocap_world', '/mocap_robot_base', rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # # assuming that the robot base is similar to the mocap_world (with pi around z-axis)
            # q_world_base = tf.transformations.quaternion_about_axis(
            #     -np.pi/2, (0, 0, 1))
            
            # self.pubish_on_point_cloud(self.pointcloud)

            # rate.sleep()

        # print('Calibration done')

        # while not self.Hand_received:
        # print('Waiting for robot..')
        #     rate.sleep()

        # print('Robot reached')
        # print('Commencing frame transformation')
        # p_hand_filtered = np.array([0, 0, 0])
        # q_hand_filtered = np.array([0, 0, 0, 0])
        # filter_factor = 1 - (1.0/freq) / (1.0/freq + 1.0/10)

        # go_to_init_possition = True
        # target_reached = False
        # init_point_reached2 = False
        # dir_pub_1 = False
        # dir_pub_2 = False
        # target_str = 'Elbow'
        # distance_to_surface = 1
        # desired_vel_distance_raw = 1
        # target_i = np.size(self.target_vec_y)-1
        # # target_i = 0
        # desired_vel_combined_old = 0
        # line_target = np.array([0, 0, 0])
        # self.svr_target = np.array([0, 0, 0])
        # gamma_target = 0.01
        # limit_real = 0.4
        # limit_simul = 0.3
        # limit = limit_real
        # limit_close = 0.07*1.5  # Change depending on the Passive DS
        # limit_close_orig = limit_close
        # limit_near=limit_close
        # # Offset since line between shoulder and hand is not completely straight
        # self.q_svr_rot_orig = tf.transformations.quaternion_about_axis(
        #     np.deg2rad(4), (0, 1, 0))
        # self.q_svr_rot_orig = tf.transformations.quaternion_multiply(
        #     tf.transformations.quaternion_about_axis(np.deg2rad(-7), (1, 0, 0)), self.q_svr_rot_orig)
        # self.q_svr_rot = tf.transformations.quaternion_multiply(
        #     self.q_svr_rot_orig, trans_arm[1])
        # self.q_svr_rot = tf.transformations.quaternion_multiply(
        #     tf.transformations.quaternion_about_axis(np.deg2rad(90), (0, 0, 1)), self.q_svr_rot)
        # self.path = Path()
        # self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        # PointCloud_i=0

        # #Used for aligning the arm.
        # q_tmp3 = tf.transformations.quaternion_about_axis(
        #         np.deg2rad(180), [0, 1, 0])
        # q_tmp2 = tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(
        #         np.deg2rad(-90), [1, 0, 0]), q_tmp3)  # lower value, facing more down    
        # q_tmp = tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(
        #         np.deg2rad(-5), [0, 1, 0]), q_tmp2)  # lower value, facing more down        
        # desired_vel_combined = np.array([0,0,0])
        print("Starting loop")
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
            # Setting palm link at the end of robot

            # world frame is the top tf published by iiwa controller
            # br_robot_base_fixed.sendTransform(trans_world_base[0], q_world_base, rospy.Time.now(),
            #                                     'world', "mocap_world")

            # # filtering the mocap_hand frame
            # try:
            #     trans_hand = self.listener.lookupTransform(
            #         '/mocap_world', '/mocap_hand', rospy.Time(0))
            #     if not self.hand_logged:
            #         rospy.loginfo("hand transform received")
            #         self.hand_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # # Filter the hand position
            # np_trans_0 = np.array(trans_hand[0])
            # np_trans_1 = np.array(trans_hand[1])
            # p_hand_filtered = (1-filter_factor) * \
            #     p_hand_filtered + filter_factor * np_trans_0
            # q_hand_filtered = (1-filter_factor) * \
            #     q_hand_filtered + filter_factor * np_trans_1
            # q_hand_filtered = q_hand_filtered / \
            #     np.linalg.norm(q_hand_filtered)

            # br_hand_filter.sendTransform(p_hand_filtered, q_hand_filtered, rospy.Time.now(),
            #                                 'mocap_hand_filtered', "mocap_world")
            # # Rotate the SVR Frame
            # self.br_svr_rotated.sendTransform(self.p_arm-[0.0, -0.01, 0.01], self.q_svr_rot, rospy.Time.now(),
            #                                     'SVR', "mocap_hand_filtered")

            # # Broacasting svr fram based on our calibration and filtered hand pose
            # br_svr.sendTransform(self.p_arm, trans_arm[1], rospy.Time.now(),
            #                         'mocap_svr', "mocap_hand_filtered")

            # # Read ee pose in svr frame
            # try:
            #     trans_svr_ee = self.listener.lookupTransform(
            #         'SVR', 'iiwa_link_ee', rospy.Time(0))
            #     if not self.ee_svr_logged:
            #         rospy.loginfo("ee_svr transform received")
            #         self.ee_svr_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # self.trans_svr_ee = trans_svr_ee
            # self.publish_svr_ee(trans_svr_ee)

            # # desired_vel_svr = -self.desired_end_vec
            # # vel_norm = np.linalg.norm(desired_vel_svr)
            # # gamma_target = 0.01
            # # limit_real = 0.2
            # # limit_simul = 0.3
            # # limit = limit_real
            # # limit_close = 0.07
            # # if(vel_norm > limit):
            # #     desired_vel_svr = desired_vel_svr / vel_norm * limit
            # # if abs(self.gamma_dist-gamma_target) < 0.05:
            # #     desired_vel_svr = desired_vel_svr * \
            # #         abs(self.gamma_dist-gamma_target)/0.05+[0, 0, 0.03]
            # desired_vel_svr_orientation = -self.desired_end_vec

            # # rospy.loginfo_throttle(1, str(self.qv_mult(trans_svr_world[1],self.desired_end_vec)))

            # # Compute the orientation of the robot
            # # q_tf = self.orientation_from_velocity(
            # #     desired_vel_svr_orientation)
           
            # # Publishing the desired orientation of the end-effector
            # try:
            #     trans_end_svr = self.listener.lookupTransform(
            #         '/SVR', '/iiwa_link_ee', rospy.Time(0))
            #     if not self.svr_ee_logged:
            #         rospy.loginfo("svr_ee transform received")
            #         self.svr_ee_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # self.trans_end_svr = np.array(trans_end_svr[0])
            # # Publishing the desired orientation of the end-effector
            # try:
            #     trans_world_svr = self.listener.lookupTransform(
            #         '/world', '/SVR', rospy.Time(0))
            #     if not self.world_svr_logged:
            #         rospy.loginfo("world_svr transform received")
            #         self.world_svr_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # self.trans_world_svr = np.array(trans_world_svr[0])
            # # br_ee_svr.sendTransform(trans_end_svr[0], q_tf, rospy.Time.now(
            # # ), 'e_desired_orientation', "SVR")
            # # Reading the desired orientation of the end-effector in the robot base frame
            # # try:
            # #     trans_ee_desired = self.listener.lookupTransform(
            # #         '/world', '/e_desired_orientation', rospy.Time(0))
            # #     if not self.ee_world_logged:
            # #         rospy.loginfo("ee_world transform received")
            # #         self.ee_world_logged = True
            # # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # #     continue
            # # Sending and reading the target in world frame
            # self.br_ee_line_target.sendTransform(line_target, [0, 0, 0, 1], rospy.Time.now(
            # ), 'line_target', "SVR")
            # try:
            #     trans_world_line = self.listener.lookupTransform(
            #         '/world', '/line_target', rospy.Time(0))
            #     if not self.world_line_logged:
            #         rospy.loginfo("world_line transform received")
            #         self.world_line_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # self.trans_world_line = np.array(trans_world_line[0])
            # self.br_ee_svr_target.sendTransform(
            #     self.svr_target, [0, 0, 0, 1], rospy.Time.now(), 'svr_target', 'SVR')
            # try:
            #     trans_world_to_svr_target = np.array(self.listener.lookupTransform(
            #         'world', 'svr_target', rospy.Time(0)))
            #     self.world_to_svr_target = np.array(
            #         trans_world_to_svr_target[0])
            #     if not self.ee_svr_target_logged:
            #         rospy.loginfo("ee_svr_target transform received")
            #         self.ee_svr_target_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # # self.trans_world_svr=np.array(trans)
            # # Getting the end-effecor in the world frame
            # try:
            #     trans_ee_real = self.listener.lookupTransform(
            #         '/world', '/iiwa_link_ee', rospy.Time(0))
            #     if not self.ee_real_logged:
            #         rospy.loginfo("ee_real transform received")
            #         self.ee_real_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # # Getting the end-effecor in the mocap_world frame
            # self.trans_ee_real = np.array(trans_ee_real[0])
            # try:
            #     trans_ee_mocap = self.listener.lookupTransform(
            #         '/mocap_world', '/iiwa_link_ee', rospy.Time(0))
            #     common_time = self.listener.getLatestCommonTime(
            #         '/palm_link', '/link_0')                    
            #     if not self.ee_mocap_logged:
            #         rospy.loginfo("ee_mocap transform received")
            #         self.ee_mocap_logged= True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue  
            # self.trans_ee_mocap = np.array(trans_ee_mocap[0])          
            # palm_rot = tf.transformations.quaternion_multiply(
            #     trans_ee_mocap[1], tf.transformations.quaternion_about_axis(np.deg2rad(180), (0, 0, 1)))
            # palm_rot2 = tf.transformations.quaternion_multiply(
            #     palm_rot, tf.transformations.quaternion_about_axis(np.deg2rad(-90), (0, 1, 0)))
            # # displacement=np.dot(tf.transformations.quaternion_matrix(palm_rot2),np.array([0,-0.0685,0.0294,0]))
            # displacement=np.dot(tf.transformations.quaternion_matrix(palm_rot2),np.array([0.0685,0,-0.0294,0]))
            # # print(np.dot(mat_tmp,np.array([0,0,0.0254,0])))
            # # test_rot=tf.transformations.rotation_matrix(0,(1,0,0))
            # # test_rot2=tf.transformations.rotation_matrix(0,(1,0,0))
            # # test_rot[0,0]=0
            # # test_rot[2,0]=1
            # # test_rot[1,1]=-1
            # # test_rot[0,2]=1
            # # test_rot[2,2]=0
            # # qt1=tf.transformations.quaternion_from_matrix(test_rot)
            # # qt2=tf.transformations.quaternion_from_matrix(test_rot2)
            # # qt3=tf.transformations.quaternion_multiply(qt1,tf.transformations.quaternion_inverse(qt2))
            # # br_palm_link.sendTransform(trans_ee_mocap[0]+np.array([0,-0.0685,0.0294]), palm_rot2,common_time,
            # # 'fake_hand_root', "mocap_world")
            # # br_palm_link.sendTransform(trans_ee_mocap[0]+np.array([0,-0.0685,0.0254]), palm_rot2,common_time,
            # # 'hand_root', "mocap_world")
            # # br_palm_link.sendTransform(trans_ee_mocap[0]+np.array([0,-0.0685,0.0294]), palm_rot2,common_time,
            # # 'hand_root', "mocap_world")
            # br_palm_link.sendTransform(trans_ee_mocap[0]+displacement[:3], palm_rot2,common_time,
            # 'hand_root', "mocap_world")
            
       

            # # Frame3: Line algorithm !WORKS OK!
            # if not init_point_reached2:
            #     line_target = np.array([0.27, 0.15, -0.02])
            #     self.forces_vec = np.array([0, 0, 0, 0, 0, 0])
            # if not init_point_reached2 and distance_to_surface < 0.07:
            #     init_point_reached2 = True
            #     print('reset')
            # # Two points
            # # if self.hand_target and init_point_reached2:
            # #     line_target = np.array([0.18, 0.25, 0.00])
            # # elif not self.hand_target and init_point_reached2:
            # #     line_target = np.array([0.18, 0.05, 0.00])
            # # if np.linalg.norm(line_target-trans_end_svr[0])<0.05 and target_reached and self.forces_received:
            # #     self.hand_target=1-self.hand_target
            # #     if self.hand_target:
            # #         target_str='Hand'
            # #     else:
            # #         target_str='Elbow'
            # # Interpolated points
            # # print(target_i)
            # if self.hand_target and init_point_reached2 and self.end_reached == False:
            #     # if dir_pub_1==False:
            #     #     counter=0
            #     #     while counter<50:
            #     #         self.DirPub.publish(-1)
            #     #         counter=counter+1
            #     #     dir_pub_1=True
            #     #     dir_pub_2=False
            #     #     print("Shoulder")
            #     line_target2 = np.array([0.16, 0.27, -0.005])
            #     line_target = np.array(
            #         [self.target_vec_x[target_i], self.target_vec_y[target_i], line_target2[2]])
            #     self.get_robot_vel(line_target)
            #     intermediate_dist = np.linalg.norm(
            #         (self.world_to_line_target-self.trans_ee_real))
            #     # intermediate_dist=np.linalg.norm(line_target-trans_end_svr[0])
            #     if intermediate_dist < 0.05:
            #         target_i = target_i-1
            #     if target_i == 0:
            #         self.end_reached = True
            # elif not self.hand_target and init_point_reached2 and self.end_reached == False:
            #     # if dir_pub_2==False:
            #     #     counter=0
            #     #     while counter<50:
            #     #         self.DirPub.publish(1)
            #     #         counter=counter+1
            #     #     dir_pub_1=False
            #     #     dir_pub_2=True
            #     #     print("Hand")
            #     line_target2 = np.array([0.18, 0.05, -0.005])
            #     line_target = np.array(
            #         [self.target_vec_x[target_i], self.target_vec_y[target_i], line_target2[2]])
            #     self.get_robot_vel(line_target)
            #     intermediate_dist = np.linalg.norm(
            #         (self.world_to_line_target-self.trans_ee_real))
            #     # intermediate_dist=np.linalg.norm(line_target-trans_end_svr[0])
            #     if intermediate_dist < 0.05:
            #         target_i = target_i+1
            #     if target_i == np.size(self.target_vec_y)-1:
            #         self.end_reached = True
            # else:
            #     self.get_robot_vel(line_target)
            # if self.end_reached and target_reached and self.forces_received:
            #     self.hand_target = 1-self.hand_target
            #     # target_i=np.size(self.target_vec_y)-target_i-1
            #     if self.hand_target:
            #         target_str = 'Hand'
            #     else:
            #         target_str = 'Elbow'
            #     self.end_reached = False
            #     # print("change direction")
            # ##
            # self.get_svm_dir(
            #     trans_end_svr[0], 0.13-(limit_close_orig-limit_near)/(20*limit_close_orig), init_point_reached2)  # 0.14 used
            # # print((limit_close_orig-limit_near)/(10*limit_close_orig))
            # # if init_point_reached2:
            # #     self.get_svm_dir(trans_end_svr[0],0.13)
            # #     # self.svm_dir=np.array([0,0,0])
            # # else:
            # #     self.svm_dir=np.array([0,0,0])
            # # Get target in robot frame
            # # print(self.world_to_line_target)
            # # line_target2 = np.array([0.15, 0.2, -0.20])

            # q_t_ee = tf.transformations.quaternion_multiply(
            #     q_tmp, self.q_svr_rot)
            # br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            # ), 'target_frame', "mocap_hand_filtered")
            # # desired_vel_distance_raw = (line_target-np.array(trans_end_svr[0])) # 7 is good, but makes noises
            # # desired_vel_distance = desired_vel_distance_raw*[-1, -1, 1]*6 # 7 is good, but makes noises
            # desired_vel_distance_raw = (
            #     self.world_to_line_target-self.trans_ee_real)
            # desired_vel_distance = desired_vel_distance_raw*20  # 6 is good
            # distance_to_surface = np.linalg.norm(desired_vel_distance_raw)
            # vel_distance_norm = np.linalg.norm(desired_vel_distance)
            # limit_near = limit_close*pow((self.forces_vec[1]+1), -3)
            # if init_point_reached2:
            #     if vel_distance_norm > limit_near:
            #         desired_vel_distance = desired_vel_distance/vel_distance_norm*limit_near
            # else:
            #     if vel_distance_norm > limit:
            #         desired_vel_distance = desired_vel_distance/vel_distance_norm*limit
            # # desired_vel_combined_tmp = (beta)*desired_vel_distance+(1-beta)*desired_vel_distance2
            # desired_vel_combined_tmp = desired_vel_distance+self.svm_dir

            # # desired_vel_combined_tmp = desired_vel_distance
            # # Get target frame for robot
            # try:
            #     trans_ee_target = self.listener.lookupTransform(
            #         'world', '/target_frame', rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # # Send end efector to good starting point for svr
            # # go_to_init_possition=False
            if self.go_to_init_possition:
                self.custom_command.data[0:3] = [0, np.deg2rad(90), 0]
                self.custom_command.data[3:6] = [0, 0, 0.03]
                t_start = time.time()
                counter = 0
                while counter < 50:
                    self.RobotCommandPub.publish(self.custom_command)
                    self.GrabPub.publish(0)
                    counter = counter+1
                    rate.sleep()
                self.go_to_init_possition = False
                raw_input('Waiting to start movement')
                self.go_to_init_possition2 = True
            if self.go_to_init_possition2:
                self.custom_command.data[0:3] = [0, np.deg2rad(140), 0]
                self.custom_command.data[3:6] = [0, 0, 0.03]
                t_start = time.time()
                counter = 0
                while counter < 50:
                    self.RobotCommandPub.publish(self.custom_command)
                    self.GrabPub.publish(0)
                    counter = counter+1
                    rate.sleep()
                self.go_to_init_possition2 = False
                raw_input('Waiting to start movement')            

            self.desired_orientation= [0, np.deg2rad(170),0]
            self.current_position = self.trans_ee_real
            direction_to_target = (self.target_position - self.current_position)
            distance_to_target = np.linalg.norm(direction_to_target)
            self.desired_velocity = self.robot_gain*(direction_to_target)
            desired_velocity_norm = np.linalg.norm(self.desired_velocity)
            if desired_velocity_norm>self.max_vel:
                self.desired_velocity=self.desired_velocity/desired_velocity_norm*self.max_vel
                
            self.custom_command.data[0:3] = self.desired_orientation
            self.custom_command.data[3:6] = self.desired_velocity
            self.RobotCommandPub.publish(self.custom_command)
            self.br_ee_target.sendTransform(self.target_position, [0,0,0,1], rospy.Time.now(
            ), 'target_position', "world")
            rospy.loginfo_throttle(1, "Max speed: "+str(np.linalg.norm(self.desired_velocity)))

            if distance_to_target<0.05 and self.attack_reached==False:
                self.target_position=self.target_init
                self.attack_reached=True

            elif distance_to_target<0.02 and self.target_reached==False and self.attack_reached==True:
                counter = 0
                while counter < 50:
                    self.GrabPub.publish(1)
                    counter = counter+1
                    rate.sleep()
                rospy.loginfo_throttle(1, ['Target reached'])
                self.target_reached = True                
             
            if self.grasped==1:
                self.target_position=self.target_init+np.array([0,0,0.3])
                rospy.loginfo_throttle(1, ['Picked up object'])


            # # q_v=self.orientation_from_velocity(desired_vel_distance)
            # # trans_ee_desired_combined = beta*np.array(trans_ee_desired[1]) + (1-beta) *q_v
            # # trans_ee_desired_combined = tf.transformations.quaternion_slerp(q_v,trans_ee_desired[1],beta)
            # # trans_ee_desired_combined=trans_ee_desired[1]
            # # q_d = np.array(trans_ee_desired[1])
            # # q_d=np.array(trans_ee_desired_combined)
            # # q_r=np.array(trans_ee_real[1])

            # # if(np.dot(q_d,q_r) < 0.0):
            # #     q_d = -q_d

            # # angles = tf.transformations.euler_from_quaternion(q_d,)
            # # quaternion_matrix : q->R

            # # Create desired velocities as a function of distance to the arm
            # # desired_vel_svr=desired_vel_svr * \
            # #     [-1, -1, 1]*np.sign(self.gamma_dist-gamma_target)

            # # rospy.loginfo_throttle(1, [str(trans_end_svr[0])])
            #     # if line_distance < 0.05:
            #     #     desired_vel_distance=desired_vel_distance * \
            #     #         line_distance/0.05 #+[0, 0, 0.03]
            # # If hand is close to arm, send grasp pose
            # if distance_to_surface <= 0.03 and target_reached == False and (target_i == np.size(self.target_vec_y)-1 or target_i == 0):
            #     counter = 0
            #     while counter < 50:
            #         self.GrabPub.publish(1)
            #         counter = counter+1
            #         rate.sleep()
            #     rospy.loginfo_throttle(1, ['Target reached'])
            #     target_reached = True
            #     # Wait for hand to start massaging
            # # If hand goes away from target, send open pose
            # elif self.gamma_dist > 0.2 and target_reached == True:
            #     counter = 0
            #     while counter < 50:
            #         self.GrabPub.publish(0)
            #         counter = counter+1
            #         rate.sleep()
            #     rospy.loginfo_throttle(1, ['Went away from target'])
            #     self.forces_vec = np.array([0, 0, 0, 0, 0, 0])
            #     target_reached = False
            #     self.forces_received = False
            #     init_point_reached2 = False
            #     self.end_reached = False
            #     self.hand_target = False
            #     target_i = np.size(self.target_vec_y)-1
            # elif init_point_reached2 == True and self.gamma_dist > 0.3:
            #     init_point_reached2 = False
            #     target_reached = False
            #     self.end_reached = False
            #     print('Start over')
            # quat_line_orientation = trans_ee_target[1]
            # angles_line_orientation = self.quat_to_direction(
            #     quat_line_orientation)
            # # q_line_tmp=tf.transformations.quaternion_matrix(trans_end_svr[1])
            # # q_line_tmp2=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(np.pi,[0,1,0]))
            # # # q_line_tmp3=tf.transformations.quaternion_multiply(q_line_tmp,q_line_tmp2)
            # # q_line_tmp3=q_line_tmp
            # # angles_line_orientation=self.quat_to_direction(tf.transformations.quaternion_from_matrix(q_line_tmp3))

            # # desired_vel_combined_tmp = (beta)*desired_vel_svr+(1-beta)*desired_vel_distance
            # # desired_vel_combined_norm=np.linalg.norm(desired_vel_combined_tmp)
            # # angles_combined = (beta)*np.array(angles) + \
            # #     (1-beta)*np.array([0, 2.5, 0])
            # # if desired_vel_combined_norm<limit:
            # #     desired_vel_combined = desired_vel_combined_tmp / desired_vel_combined_norm * limit
            # # else:
            # #     desired_vel_combined = desired_vel_combined_tmp

            # desired_vel_combined = desired_vel_combined_tmp
            # # if init_point_reached2 and np.linalg.norm((desired_vel_combined-desired_vel_combined_old))>0.1:
            # #     desired_vel_combined=(desired_vel_combined+desired_vel_combined_old)/2
            # #     print('cutoff')
            # # if init_point_reached2 and np.dot(desired_vel_combined,desired_vel_combined_old)<0:
            # #     desired_vel_combined=np.array([0,0,0])
            # #     print("dot")
            # desired_vel_combined = 0.2*desired_vel_combined_tmp+0.8*desired_vel_combined_old
            # desired_vel_combined_old = desired_vel_combined

            # # rospy.loginfo_throttle(0.5, ['line_dist '+str(np.round(distance_to_surface,7))])
            # # rospy.loginfo_throttle(1, ['Beta  '+str(np.round(beta,3))])
            # # rospy.loginfo_throttle(0.5, ['Forces  '+str(self.forces_vec)])
            # # rospy.loginfo_throttle(0.5, ['gamma  '+str(self.gamma_dist)])
            # # rospy.loginfo_throttle(0.5, ['SVM   '+str(self.svm_dir)])
            # # rospy.loginfo_throttle(0.5, ['Desired vel '+str(desired_vel_combined)])
            # # rospy.loginfo_throttle(0.5, ['Line target '+str(self.world_to_line_target)])
            # # rospy.loginfo_throttle(0.5, ['Bools '+str(init_point_reached2)+str(self.end_reached)+str(self.forces_received)])

            # # rospy.loginfo_throttle(1, ['SVR'+str(trans_svr_ee[0])])
            # br_ee_debug_svr.sendTransform(
            #     self.svm_dir+self.trans_ee_real, [0, 0, 0, 1], rospy.Time.now(), 'Debug_SVR', 'world')
            # br_ee_debug_com.sendTransform(
            #     desired_vel_combined+self.trans_ee_real, [0, 0, 0, 1], rospy.Time.now(), 'Debug_Com', 'world')

            # # rospy.loginfo_throttle(0.5, ['Target   '+target_str+str(target_reached)+ str(self.forces_received)])
            # rospy.loginfo_throttle(0.5, ["limit "+str(limit_near)])
            # # rospy.loginfo_throttle(0.5, ["dist "+str(desired_vel_distance)])
            # # rospy.loginfo_throttle(1, ['Alpha '+str(np.round(alpha,3))])
            # # rospy.loginfo_throttle(1, ['Alpha vel '+str((beta*desired_vel_distance+(1-beta)*desired_vel_distance2))])
            # # rospy.loginfo_throttle(1, ['Beta  vel '+str(desired_vel_distance3)])
            rospy.loginfo_throttle(1, ['Dist to target '+str(distance_to_target)])            

            # self.custom_command.data[0:3] = angles_line_orientation
            # # self.custom_command.data[0:3] = [0,np.pi,0]#[0,2,0]
            # self.custom_command.data[3:6] = desired_vel_combined
            # # self.custom_command.data = desired_vel_svr
            # self.RobotCommandPub.publish(self.custom_command)

            # self.trans_end_svr = trans_end_svr

            # # Publish pointcloud of arm
            # if PointCloud_i>10:
            #     self.pubish_on_point_cloud(self.pointcloud)
            #     PointCloud_i=0
            # PointCloud_i=PointCloud_i+1
            # # self.publish_path()

            # # Publish transformation of Thumb // Not necesseary, published in finger_motion_generator
            # # try:
            # #     ee_to_finger = self.listener.lookupTransform(
            # #         'iiwa_link_ee', 'Finger 3', rospy.Time(0))
            # # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # #     continue
            # # br_svr.sendTransform(ee_to_finger[0], ee_to_finger[1], rospy.Time.now(),
            # #                      'finger_tran', "world")

            # # if (np.linalg.norm(self.svm_dir)!=0):
            # q_tf = self.quat_from_Vector(
            #     -self.desired_end_vec)
            # self.quatPosePub.position.x=self.trans_ee_real[0]
            # self.quatPosePub.position.y=self.trans_ee_real[1]
            # self.quatPosePub.position.z=self.trans_ee_real[2]
            # self.quatPosePub.orientation.x=q_tf[0]
            # self.quatPosePub.orientation.y=q_tf[1]
            # self.quatPosePub.orientation.z=q_tf[2]
            # self.quatPosePub.orientation.w=q_tf[3]
            # self.SVRQuatPub.publish(self.quatPosePub)
            #     # br_ee_debug_quat_com.sendTransform(trans_svr_ee[0],q_tf, rospy.Time.now(),'svr quat','SVR')
            # q_tf2=self.quat_from_Vector(desired_vel_combined)
            # self.totPosePub.position.x=self.trans_ee_real[0]
            # self.totPosePub.position.y=self.trans_ee_real[1]
            # self.totPosePub.position.z=self.trans_ee_real[2]
            # self.totPosePub.orientation.x=q_tf2[0]
            # self.totPosePub.orientation.y=q_tf2[1]
            # self.totPosePub.orientation.z=q_tf2[2]
            # self.totPosePub.orientation.w=q_tf2[3]
            # self.TotQuatPub.publish(self.totPosePub)
            # br_ee_debug_quat_com.sendTransform(self.trans_ee_real,q_tf2, rospy.Time.now(),'tot quat','world')
            rate.sleep()

    def get_robot_vel(self, line_target):
        # transformed=False

        # while not transformed:
        #     self.listener.waitForTransform('/world', '/SVR', rospy.Time.now(), rospy.Duration(4.0))
        #     try:
        #         trans_world_to_line_target = np.array(self.listener.lookupTransform('/world', '/SVR', rospy.Time()))
        # self.world_to_line_target = self.trans_world_svr + np.array(line_target)
        self.world_to_line_target = self.trans_world_line
        # self.world_to_line_target = self.trans_world_svr
        # self.br_ee_line_target.sendTransform(line_target,[0,0,0,1], rospy.Time.now(),'line_target','SVR')
        # self.br_ee_line_target2.sendTransform(self.world_to_line_target,[0,0,0,1], rospy.Time.now(),'line_target2','world')

        #     transformed = True
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        # self.br_ee_line_target.sendTransform(line_target,[0,0,0,1], rospy.Time.now(),'line_target','SVR')
        # self.listener.waitForTransform('/world', '/line_target', rospy.Time.now(), rospy.Duration(4.0))
        # try:
        #     # self.listener.waitForTransform('/world', '/line_target', now, rospy.Duration(1.0))
        #     trans_world_to_line_target = np.array(self.listener.lookupTransform(
        #         '/world', '/line_target', rospy.Time(0)))
        #     self.world_to_line_target=np.array(trans_world_to_line_target[0])
        #     transformed=True
        #     if not self.ee_line_target_logged:
        #         rospy.loginfo("ee_line_target transform received")
        #         self.ee_line_target_logged=True
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     a=1
        #     # print('SLeep')
        #     # rospy.sleep(0.1)
        #     continue

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

    def init_topics(self):
        self.RobotCommandPub = rospy.Publisher(
            "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)        
        self.GrabPub = rospy.Publisher(
            "/grab", Int8, queue_size=3)
        self.GrabPub = rospy.Publisher(
             "/grab", Int8, queue_size=3)
        self.GraspedSub = rospy.Subscriber(
            "/grasped", Int8, self.chatterCallback_Grasped)

        # self.GammaSub = rospy.Subscriber(
        #     "/svr/gamma", Float64, self.chatterCallback_Gamma)
        # self.RobotPosDesiredSub = rospy.Subscriber(
        #     "/convert_tf/desired_vel_in_svr", TwistStamped, self.chatterCallback_desiredVel)
        # self.GammaSub = rospy.Subscriber(
        #     "/thumb_forces", Float64MultiArray, self.chatterCallback_Forces)
        # self.RobotPosConvPub = rospy.Publisher(
        #     "/convert_tf/ee_in_svr", Pose, queue_size=3)
        # self.RobotCommandPub = rospy.Publisher(
        #     "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)
        # self.CloudPub = rospy.Publisher(
        #     "/PointCloud/points", PointCloud)
        # self.GrabPub = rospy.Publisher(
        #     "/grab", Int8, queue_size=3)
        # self.DirPub = rospy.Publisher(
        #     "/direction", Int8, queue_size=3)
        # self.SVRQuatPub = rospy.Publisher(
        #     "/SVRQuatPub", Pose, queue_size=3)
        # self.TotQuatPub = rospy.Publisher(
        #     "/TotQuatPub", Pose, queue_size=3)

        # # Debug publishers
        # self.DebugVelPub = rospy.Publisher(
        #     "/debug/vel_combined", Pose, queue_size=3)
        # self.DebugSvrPub = rospy.Publisher(
        #     "/debug/svr_combined", Pose, queue_size=3)

    def init_params(self):
        # Initial position
        self.go_to_init_possition = True
        self.go_to_init_possition2 = False
        self.custom_command = Float64MultiArray()
        self.custom_command.data = [0, 0, 0, 0, 0, 0]

        # Target
        self.target_init = np.array([0.55,0,0.34])
        self.attack_position = self.target_init + np.array([0,0,0.2])
        self.target_position = self.attack_position
        self.target_reached = False
        self.attack_reached = False

        # End effector transform
        self.ee_svr_logged = False

        # Robot parameters
        self.max_vel = 0.2
        self.robot_gain = 4

        # Grasped
        self.grasped = 0
    #     self.desired_end_received = False
    #     self.Hand_received = False
    #     self.end = Pose()
    #     self.ee_in_svr = Pose()
    #     self.trans_arm = []
    #     # rx ry rz and vx vy vz (first desired orientation and then desired velocity)
    #     self.custom_command = Float64MultiArray()
    #     self.forces_vec = np.array([0, 0, 0, 0, 0, 0])
    #     self.svm_dir = np.array([0, 0, 0])

    #     self.hand_target = False
    #     self.custom_command.data = [0, 0, 0, 0, 0, 0]
    #     self.forces_received = False

    #     self.target_vec_x = [
    #         float(i)/10000-0.02 for i in range(1800, 1549, -29)]
    #     self.target_vec_y = [0.05, 0.075, 0.1,
    #                             0.135, 0.17, 0.195, 0.24, 0.28, 0.28]
    #     self.target_vec_y2 = self.target_vec_y[::-1]
    #     self.end_reached = False

    #     # Debug
    #     self.debugVelPose = Pose()
    #     self.debugSvrPose = Pose()

    #     # Logging transforms
    #     self.hand_logged = False
    #     self.ee_svr_logged = False
    #     self.svr_ee_logged = False
    #     self.ee_world_logged = False
    #     self.ee_real_logged = False
    #     self.ee_line_target_logged = False
    #     self.ee_svr_target_logged = False
    #     self.ee_svr_target_logged2 = False
    #     self.world_svr_logged = False
    #     self.world_line_logged = False
    #     self.ee_mocap_logged = False  

    #     #Publish SVR quat
    #     self.quatPosePub =Pose()
    #     self.totPosePub = Pose()

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
    convert_tf()
