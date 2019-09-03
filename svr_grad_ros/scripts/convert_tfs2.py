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


class convert_tf():
    def __init__(self):

        freq = 200
        rospy.init_node('convert_tf', anonymous=True)

        self.listener = tf.TransformListener()
        br_svr = tf.TransformBroadcaster()
        br_ee = tf.TransformBroadcaster()
        br_ee_conv = tf.TransformBroadcaster()
        br_robot_base_fixed = tf.TransformBroadcaster()
        self.br_svr_rotated = tf.TransformBroadcaster()
        br_hand_filter = tf.TransformBroadcaster()
        br_ee_svr = tf.TransformBroadcaster()
        br_ee_target = tf.TransformBroadcaster()
        br_ee_finger = tf.TransformBroadcaster()
        self.br_ee_line_target = tf.TransformBroadcaster()
        self.br_ee_line_target2 = tf.TransformBroadcaster()
        self.br_ee_svr_target = tf.TransformBroadcaster()
        br_ee_debug_svr =tf.TransformBroadcaster()
        br_ee_debug_com =tf.TransformBroadcaster()

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        # br_robot_base_world_mocap = tf.TransformBroadcaster()


        self.init_topics()
        self.init_params()

        rate = rospy.Rate(freq)
        start = time.time()

        # calibration of the arm orientation
        print('Calibrating the arm orientation')
        delay = 1
        while time.time() < start+delay and not rospy.is_shutdown():
            try:
                trans_arm = self.listener.lookupTransform(
                    '/mocap_hand', '/mocap_shoulder', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_arm = trans_arm
            self.p_arm = np.array(
                [trans_arm[0][0], trans_arm[0][1], trans_arm[0][2]])

            self.p_arm = self.p_arm/1.7

            br_svr.sendTransform(self.p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand")

            # freezing the robot_base
            try:
                trans_world_base = self.listener.lookupTransform(
                    '/mocap_world', '/mocap_robot_base', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # assuming that the robot base is similar to the mocap_world (with pi around z-axis)
            q_world_base = tf.transformations.quaternion_about_axis(
                -np.pi/2, (0, 0, 1))

            rate.sleep()

        print('Calibration done')

        # while not self.Hand_received:
        # print('Waiting for robot..')
        #     rate.sleep()

        print('Robot reached')
        print('Commencing frame transformation')
        p_hand_filtered = np.array([0, 0, 0])
        q_hand_filtered = np.array([0, 0, 0, 0])
        filter_factor = 1 - (1.0/freq) / (1.0/freq + 1.0/10)

        go_to_init_possition = True
        target_reached = False
        init_point_reached2 = False
        target_str='Elbow'
        distance_to_surface=1
        desired_vel_distance_raw=1
        # target_i=np.size(self.target_vec_y)-1
        target_i=0
        desired_vel_combined_old=0
        line_target=np.array([0,0,0])
        self.svr_target=np.array([0,0,0])
        while not rospy.is_shutdown():
            # world frame is the top tf published by iiwa controller
            br_robot_base_fixed.sendTransform(trans_world_base[0], q_world_base, rospy.Time.now(),
                                              'world', "mocap_world")

            # filtering the mocap_hand frame
            try:
                trans_hand = self.listener.lookupTransform(
                    '/mocap_world', '/mocap_hand', rospy.Time(0))
                if not self.hand_logged:
                    rospy.loginfo("hand transform received")
                    self.hand_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Filter the hand position
            np_trans_0 = np.array(trans_hand[0])
            np_trans_1 = np.array(trans_hand[1])
            p_hand_filtered = (1-filter_factor) * \
                p_hand_filtered + filter_factor * np_trans_0
            q_hand_filtered = (1-filter_factor) * \
                q_hand_filtered + filter_factor * np_trans_1
            q_hand_filtered = q_hand_filtered / np.linalg.norm(q_hand_filtered)

            br_hand_filter.sendTransform(p_hand_filtered, q_hand_filtered, rospy.Time.now(),
                                         'mocap_hand_filtered', "mocap_world")
            # Rotate the SVR Frame
            self.q_svr_rot = tf.transformations.quaternion_about_axis(
                np.deg2rad(0), (1, 0, 0))
            self.q_svr_rot = tf.transformations.quaternion_multiply(
                self.q_svr_rot, trans_arm[1])
            self.q_svr_rot = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_about_axis(np.deg2rad(90), (0, 0, 1)), self.q_svr_rot)
            self.br_svr_rotated.sendTransform(self.p_arm-[0.0, -0.01, 0.01], self.q_svr_rot, rospy.Time.now(),
                                         'SVR', "mocap_hand_filtered")

            # Broacasting svr fram based on our calibration and filtered hand pose
            br_svr.sendTransform(self.p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand_filtered")

            # Read ee pose in svr frame
            try:
                trans_svr_ee = self.listener.lookupTransform(
                    'SVR', 'iiwa_link_ee', rospy.Time(0))
                if not self.ee_svr_logged:
                    rospy.loginfo("ee_svr transform received")
                    self.ee_svr_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.trans_svr_ee=trans_svr_ee
            self.publish_svr_ee(trans_svr_ee)

            desired_vel_svr = -self.desired_end_vec

            vel_norm = np.linalg.norm(desired_vel_svr)
            gamma_target = 0.01
            limit_real = 0.2
            limit_simul = 0.3
            limit = limit_real
            limit_close = 0.07
            # if(vel_norm > limit):
            #     desired_vel_svr = desired_vel_svr / vel_norm * limit
            # if abs(self.gamma_dist-gamma_target) < 0.05:
            #     desired_vel_svr = desired_vel_svr * \
            #         abs(self.gamma_dist-gamma_target)/0.05+[0, 0, 0.03]
            desired_vel_svr_orientation = -self.desired_end_vec

            # rospy.loginfo_throttle(1, str(self.qv_mult(trans_svr_world[1],self.desired_end_vec)))

            # Compute the orientation of the robot
            q_tf = self.orientation_from_velocity(desired_vel_svr_orientation)

            # Publishing the desired orientation of the end-effector
            try:
                trans_end_svr = self.listener.lookupTransform(
                    '/SVR', '/iiwa_link_ee', rospy.Time(0))
                if not self.svr_ee_logged:
                    rospy.loginfo("svr_ee transform received")
                    self.svr_ee_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_end_svr=np.array(trans_end_svr[0])
            # Publishing the desired orientation of the end-effector
            try:
                trans_world_svr = self.listener.lookupTransform(
                    '/world', '/SVR', rospy.Time(0))
                if not self.world_svr_logged:
                    rospy.loginfo("world_svr transform received")
                    self.world_svr_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_world_svr=np.array(trans_world_svr[0])
            br_ee_svr.sendTransform(trans_end_svr[0], q_tf, rospy.Time.now(
            ), 'e_desired_orientation', "SVR")
            # Reading the desired orientation of the end-effector in the robot base frame
            try:
                trans_ee_desired = self.listener.lookupTransform(
                    '/world', '/e_desired_orientation', rospy.Time(0))
                if not self.ee_world_logged:
                    rospy.loginfo("ee_world transform received")
                    self.ee_world_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # Sending and reading the target in world frame
            self.br_ee_line_target.sendTransform(line_target, [0,0,0,1], rospy.Time.now(
            ), 'line_target', "SVR")
            try:
                trans_world_line = self.listener.lookupTransform(
                    '/world', '/line_target', rospy.Time(0))
                if not self.world_line_logged:
                    rospy.loginfo("world_line transform received")
                    self.world_line_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue           
            self.trans_world_line=np.array(trans_world_line[0])
            self.br_ee_svr_target.sendTransform(self.svr_target,[0,0,0,1], rospy.Time.now(),'svr_target','SVR')
            try:
                trans_world_to_svr_target = np.array(self.listener.lookupTransform(
                    'world', 'svr_target', rospy.Time(0)))
                self.world_to_svr_target=np.array(trans_world_to_svr_target[0])
                if not self.ee_svr_target_logged:
                    rospy.loginfo("ee_svr_target transform received")
                    self.ee_svr_target_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue    
            # self.trans_world_svr=np.array(trans)            
            try:
                trans_ee_real = self.listener.lookupTransform(
                    '/world', '/iiwa_link_ee', rospy.Time(0))
                if not self.ee_real_logged:
                    rospy.loginfo("ee_real transform received")
                    self.ee_real_logged=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_ee_real=np.array(trans_ee_real[0])
            # Create target frame for robot ee
            # Frame1: Horizontal
            # line_target = np.array([-0.04, 0.2, 0.15]) # For movements on top of arm
            # q_tmp=tf.transformations.quaternion_about_axis(np.deg2rad(180),[0,1,0])
            # q_t_ee=tf.transformations.quaternion_multiply(q_tmp,self.q_svr_rot)
            # desired_vel_distance = (line_target-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # distance_to_surface=np.linalg.norm(desired_vel_distance)
            # if distance_to_surface > limit:
            #     desired_vel_distance=desired_vel_distance/distance_to_surface*limit
            # beta=1/(1+np.exp((distance_to_surface-0.01)/0.01))
            # br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            # ), 'target_frame', "mocap_hand_filtered")
            # desired_vel_combined_tmp = desired_vel_distance

            # Frame2: Vertical
            # line_target = np.array([0.15, 0.2, 0.00]) # For movements on the side of the arm
            # line_target2 = np.array([0.15, 0.2, -0.20])
            # q_tmp2=tf.transformations.quaternion_about_axis(np.deg2rad(180),[0,1,0])
            # q_tmp=tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(np.deg2rad(-90),[1,0,0]),q_tmp2)
            # q_t_ee=tf.transformations.quaternion_multiply(q_tmp,self.q_svr_rot)
            # br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            # ), 'target_frame', "mocap_hand_filtered")
            # desired_vel_distance = (line_target-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # desired_vel_distance2 = (line_target2-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # distance_to_surface=np.linalg.norm(desired_vel_distance)
            # if distance_to_surface > limit:
            #     desired_vel_distance=desired_vel_distance/distance_to_surface*limit
            # distance_to_surface2=np.linalg.norm(desired_vel_distance2)
            # if distance_to_surface2 > limit:
            #     desired_vel_distance2=desired_vel_distance2/distance_to_surface2*limit
            # beta_distance=np.linalg.norm(line_target-np.array(trans_end_svr[0]))
            # beta=1/(1+np.exp((beta_distance-0.2)/0.01))
            # desired_vel_combined_tmp = (beta)*desired_vel_distance+(1-beta)*desired_vel_distance2

            # Frame3: Line algorithm !DOES NOT WORK!
            # line_target = np.array([0.15, 0.2, 0.00]) # For movements on the side of the arm
            # line_target2 = np.array([0.15, 0.2, -0.20])
            # if self.hand_target:
            #     line_target3 = np.array([0.15, 0.3, 0.00])
            # elif not self.hand_target:
            #     line_target3 = np.array([0.15, 0.0, 0.00])
            # if np.linalg.norm(line_target3-trans_end_svr[0])<0.05:
            #     self.hand_target=1-self.hand_target
            # q_tmp2=tf.transformations.quaternion_about_axis(np.deg2rad(180),[0,1,0])
            # q_tmp=tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(np.deg2rad(-90),[1,0,0]),q_tmp2)
            # q_t_ee=tf.transformations.quaternion_multiply(q_tmp,self.q_svr_rot)
            # br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            # ), 'target_frame', "mocap_hand_filtered")
            # desired_vel_distance = (line_target-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # desired_vel_distance2 = (line_target2-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # desired_vel_distance3 = (line_target3-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # distance_to_surface=np.linalg.norm(desired_vel_distance)
            # if distance_to_surface > limit:
            #     desired_vel_distance=desired_vel_distance/distance_to_surface*limit
            # distance_to_surface2=np.linalg.norm(desired_vel_distance2)
            # if distance_to_surface2 > limit:
            #     desired_vel_distance2=desired_vel_distance2/distance_to_surface2*limit
            # distance_to_surface3=np.linalg.norm(desired_vel_distance3)
            # if distance_to_surface3 > limit/2:
            #     desired_vel_distance3=desired_vel_distance3/distance_to_surface3*limit
            # beta_distance=np.linalg.norm(line_target-np.array(trans_end_svr[0]))
            # beta=1/(1+np.exp((beta_distance-0.2)/0.01))
            # alpha=1/(1+np.exp((beta_distance-0.06)/0.005))
            # desired_vel_combined_tmp = (1-alpha)*((beta)*desired_vel_distance+(1-beta)*desired_vel_distance2)+(alpha)*desired_vel_distance3
            # desired_vel_combined_tmp = (beta)*desired_vel_distance+(1-beta)*desired_vel_distance2

            # Frame3: Line algorithm !WORKS OK!
            # if self.hand_target:
            #     line_target = np.array([0.15, 0.3, 0.00])
            # elif not self.hand_target:
            #     line_target = np.array([0.15, 0.0, 0.00])
            # if np.linalg.norm(line_target-trans_end_svr[0])<0.05:
            #     self.hand_target=1-self.hand_target
            # line_target2 = np.array([0.15, 0.2, -0.20])
            # q_tmp2=tf.transformations.quaternion_about_axis(np.deg2rad(180),[0,1,0])
            # q_tmp=tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(np.deg2rad(-90),[1,0,0]),q_tmp2)
            # q_t_ee=tf.transformations.quaternion_multiply(q_tmp,self.q_svr_rot)
            # br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            # ), 'target_frame', "mocap_hand_filtered")
            # desired_vel_distance = (line_target-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # desired_vel_distance2 = (line_target2-np.array(trans_end_svr[0]))*[-1, -1, 1]*5 # 7 is good, but makes noises
            # distance_to_surface=np.linalg.norm(desired_vel_distance)
            # limit_near=limit*pow((self.forces_vec[0]+1),-3)
            # if distance_to_surface > limit_near:
            #     desired_vel_distance=desired_vel_distance/distance_to_surface*limit_near
            # rospy.loginfo_throttle(1,["limit "+str(limit_near)])
            # rospy.loginfo_throttle(1,["dist "+str(desired_vel_distance)])
            # distance_to_surface2=np.linalg.norm(desired_vel_distance2)
            # if distance_to_surface2 > limit:
            #     desired_vel_distance2=desired_vel_distance2/distance_to_surface2*limit
            # beta_distance=np.linalg.norm(np.array([0.15, 0.2, 0.00])-np.array(trans_end_svr[0]))
            # beta=1/(1+np.exp((beta_distance-0.2)/0.01))
            # desired_vel_combined_tmp = (beta)*desired_vel_distance+(1-beta)*desired_vel_distance2

            # Frame3: Line algorithm !WORKS OK!
            if not init_point_reached2:
                line_target = np.array([0.25, 0.15, 0.00])
                self.forces_vec = np.array([0,0,0,0,0,0])
            if not init_point_reached2 and distance_to_surface<0.07:
                init_point_reached2=True
                print('reset')
            # Two points
            # if self.hand_target and init_point_reached2:
            #     line_target = np.array([0.18, 0.25, 0.00])
            # elif not self.hand_target and init_point_reached2:
            #     line_target = np.array([0.18, 0.05, 0.00])
            # if np.linalg.norm(line_target-trans_end_svr[0])<0.05 and target_reached and self.forces_received:
            #     self.hand_target=1-self.hand_target
            #     if self.hand_target:
            #         target_str='Hand'
            #     else:
            #         target_str='Elbow'
            # Interpolated points
            # print(target_i)
            if self.hand_target and init_point_reached2 and self.end_reached==False:
                line_target2 = np.array([0.16, 0.25, -0.00])
                line_target=np.array([self.target_vec_x[target_i],self.target_vec_y[target_i],line_target2[2]])
                self.get_robot_vel(line_target)
                intermediate_dist=np.linalg.norm((self.world_to_line_target-self.trans_ee_real))
                # intermediate_dist=np.linalg.norm(line_target-trans_end_svr[0])
                if intermediate_dist<0.05:
                    target_i=target_i-1
                if target_i==0:
                    self.end_reached=True
            elif not self.hand_target and init_point_reached2 and self.end_reached==False:
                line_target2 = np.array([0.18, 0.05, -0.00])
                line_target=np.array([self.target_vec_x[target_i],self.target_vec_y[target_i],line_target2[2]])
                self.get_robot_vel(line_target)
                intermediate_dist=np.linalg.norm((self.world_to_line_target-self.trans_ee_real))
                # intermediate_dist=np.linalg.norm(line_target-trans_end_svr[0])
                if intermediate_dist<0.05:
                    target_i=target_i+1
                if target_i==np.size(self.target_vec_y)-1:
                    self.end_reached=True
            else:
                self.get_robot_vel(line_target)
            if self.end_reached and target_reached and self.forces_received:
                self.hand_target=1-self.hand_target
                # target_i=np.size(self.target_vec_y)-target_i-1
                if self.hand_target:
                    target_str='Hand'
                else:
                    target_str='Elbow'
                self.end_reached=False            
            ##
            self.get_svm_dir(trans_end_svr[0],0.14,init_point_reached2)

            # if init_point_reached2:
            #     self.get_svm_dir(trans_end_svr[0],0.13)
            #     # self.svm_dir=np.array([0,0,0])
            # else:
            #     self.svm_dir=np.array([0,0,0])
            # Get target in robot frame
            # print(self.world_to_line_target)
            # line_target2 = np.array([0.15, 0.2, -0.20])
            q_tmp2=tf.transformations.quaternion_about_axis(np.deg2rad(180),[0,1,0])
            q_tmp=tf.transformations.quaternion_multiply(tf.transformations.quaternion_about_axis(np.deg2rad(-90),[1,0,0]),q_tmp2)
            q_t_ee=tf.transformations.quaternion_multiply(q_tmp,self.q_svr_rot)
            br_ee_target.sendTransform(self.p_arm-[0.0, -0.01, 0.01], q_t_ee, rospy.Time.now(
            ), 'target_frame', "mocap_hand_filtered")
            # desired_vel_distance_raw = (line_target-np.array(trans_end_svr[0])) # 7 is good, but makes noises
            # desired_vel_distance = desired_vel_distance_raw*[-1, -1, 1]*6 # 7 is good, but makes noises
            desired_vel_distance_raw = ( self.world_to_line_target-self.trans_ee_real)
            desired_vel_distance = desired_vel_distance_raw*20 # 6 is good
            distance_to_surface=np.linalg.norm(desired_vel_distance_raw)
            vel_distance_norm = np.linalg.norm(desired_vel_distance)
            limit_near=limit_close*pow((self.forces_vec[1]+1),-3)
            if init_point_reached2:
                if vel_distance_norm > limit_near:
                    desired_vel_distance=desired_vel_distance/vel_distance_norm*limit_near
            else:
                if vel_distance_norm > limit:
                    desired_vel_distance=desired_vel_distance/vel_distance_norm*limit
            # desired_vel_combined_tmp = (beta)*desired_vel_distance+(1-beta)*desired_vel_distance2
            desired_vel_combined_tmp = desired_vel_distance+self.svm_dir

            # desired_vel_combined_tmp = desired_vel_distance     
            # Get target frame for robot
            try:
                trans_ee_target = self.listener.lookupTransform(
                    'world', '/target_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # Send end efector to good starting point for svr
            # go_to_init_possition=False
            if go_to_init_possition:
                self.custom_command.data[0:3] = [0,np.deg2rad(90),0]
                self.custom_command.data[3:6] = [0,0,0.03]
                t_start=time.time()
                counter=0
                while counter<50:
                    self.RobotCommandPub.publish(self.custom_command)
                    self.GrabPub.publish(0)
                    counter=counter+1
                    rate.sleep()
                go_to_init_possition = False
                raw_input('Waiting to start movement')

            # q_v=self.orientation_from_velocity(desired_vel_distance)
            # trans_ee_desired_combined = beta*np.array(trans_ee_desired[1]) + (1-beta) *q_v
            # trans_ee_desired_combined = tf.transformations.quaternion_slerp(q_v,trans_ee_desired[1],beta)
            # trans_ee_desired_combined=trans_ee_desired[1]
            # q_d = np.array(trans_ee_desired[1])
            # q_d=np.array(trans_ee_desired_combined)
            # q_r=np.array(trans_ee_real[1])

            # if(np.dot(q_d,q_r) < 0.0):
            #     q_d = -q_d

            # angles = tf.transformations.euler_from_quaternion(q_d,)
            # quaternion_matrix : q->R


            # Create desired velocities as a function of distance to the arm
            # desired_vel_svr=desired_vel_svr * \
            #     [-1, -1, 1]*np.sign(self.gamma_dist-gamma_target)


            # rospy.loginfo_throttle(1, [str(trans_end_svr[0])])
                # if line_distance < 0.05:
                #     desired_vel_distance=desired_vel_distance * \
                #         line_distance/0.05 #+[0, 0, 0.03]
            # If hand is close to arm, send grasp pose
            if distance_to_surface <= 0.03 and target_reached==False and (target_i==np.size(self.target_vec_y)-1 or target_i==0):
                counter=0
                while counter<50:
                    self.GrabPub.publish(1)
                    counter=counter+1
                    rate.sleep()
                rospy.loginfo_throttle(1, ['Target reached'])
                target_reached=True
                # Wait for hand to start massaging
            # If hand goes away from target, send open pose
            elif self.gamma_dist > 0.2 and target_reached==True:
                counter=0
                while counter<50:
                    self.GrabPub.publish(0)
                    counter=counter+1
                    rate.sleep()
                rospy.loginfo_throttle(1, ['Went away from target'])
                self.forces_vec = np.array([0,0,0,0,0,0])
                target_reached=False
                self.forces_received=False
                init_point_reached2=False
                self.end_reached=False
            elif init_point_reached2==True and self.gamma_dist>0.3:
                init_point_reached2=False
                target_reached=False
                self.end_reached=False
                print('Start over')
            quat_line_orientation = trans_ee_target[1]
            angles_line_orientation = self.quat_to_direction(quat_line_orientation)
            # q_line_tmp=tf.transformations.quaternion_matrix(trans_end_svr[1])
            # q_line_tmp2=tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(np.pi,[0,1,0]))
            # # q_line_tmp3=tf.transformations.quaternion_multiply(q_line_tmp,q_line_tmp2)
            # q_line_tmp3=q_line_tmp
            # angles_line_orientation=self.quat_to_direction(tf.transformations.quaternion_from_matrix(q_line_tmp3))

            # desired_vel_combined_tmp = (beta)*desired_vel_svr+(1-beta)*desired_vel_distance
            # desired_vel_combined_norm=np.linalg.norm(desired_vel_combined_tmp)
            # angles_combined = (beta)*np.array(angles) + \
            #     (1-beta)*np.array([0, 2.5, 0])
            # if desired_vel_combined_norm<limit:
            #     desired_vel_combined = desired_vel_combined_tmp / desired_vel_combined_norm * limit
            # else:
            #     desired_vel_combined = desired_vel_combined_tmp

            desired_vel_combined = desired_vel_combined_tmp
            # if init_point_reached2 and np.linalg.norm((desired_vel_combined-desired_vel_combined_old))>0.1:
            #     desired_vel_combined=(desired_vel_combined+desired_vel_combined_old)/2
            #     print('cutoff')
            # if init_point_reached2 and np.dot(desired_vel_combined,desired_vel_combined_old)<0:
            #     desired_vel_combined=np.array([0,0,0])
            #     print("dot")
            desired_vel_combined=0.2*desired_vel_combined_tmp+0.8*desired_vel_combined_old
            desired_vel_combined_old=desired_vel_combined

            # rospy.loginfo_throttle(0.5, ['line_dist '+str(np.round(distance_to_surface,7))])
            # rospy.loginfo_throttle(1, ['Beta  '+str(np.round(beta,3))])
            # rospy.loginfo_throttle(0.5, ['Forces  '+str(self.forces_vec)])
            rospy.loginfo_throttle(0.5, ['gamma  '+str(self.gamma_dist)])
            rospy.loginfo_throttle(0.5, ['SVM   '+str(self.svm_dir)])
            rospy.loginfo_throttle(0.5, ['Desired vel '+str(desired_vel_combined)]) 
            # rospy.loginfo_throttle(0.5, ['Line target '+str(self.world_to_line_target)])
            rospy.loginfo_throttle(0.5, ['Bools '+str(init_point_reached2)+str(self.end_reached)+str(self.forces_received)])

            # rospy.loginfo_throttle(1, ['SVR'+str(trans_svr_ee[0])])
            br_ee_debug_svr.sendTransform(self.svm_dir+self.trans_ee_real,[0,0,0,1],rospy.Time.now(),'Debug_SVR','world')
            br_ee_debug_com.sendTransform(desired_vel_combined+self.trans_ee_real,[0,0,0,1],rospy.Time.now(),'Debug_Com','world')


            # rospy.loginfo_throttle(0.5, ['Target   '+target_str+str(target_reached)+ str(self.forces_received)])
            # rospy.loginfo_throttle(0.5, ["limit "+str(limit_near)])
            # rospy.loginfo_throttle(0.5, ["dist "+str(desired_vel_distance)])
            # rospy.loginfo_throttle(1, ['Alpha '+str(np.round(alpha,3))])
            # rospy.loginfo_throttle(1, ['Alpha vel '+str((beta*desired_vel_distance+(1-beta)*desired_vel_distance2))])
            # rospy.loginfo_throttle(1, ['Beta  vel '+str(desired_vel_distance3)])

            self.custom_command.data[0:3] = angles_line_orientation
            # self.custom_command.data[0:3] = [0,np.pi,0]#[0,2,0]
            self.custom_command.data[3:6] = desired_vel_combined
            # self.custom_command.data = desired_vel_svr
            self.RobotCommandPub.publish(self.custom_command)

            self.trans_end_svr=trans_end_svr

            # Publish pointcloud of arm
            self.load_pointcloud()
            self.pubish_on_point_cloud(self.pointcloud)

            # Publish transformation of Thumb // Not necesseary, published in finger_motion_generator
            # try:
            #     ee_to_finger = self.listener.lookupTransform(
            #         'iiwa_link_ee', 'Finger 3', rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # br_svr.sendTransform(ee_to_finger[0], ee_to_finger[1], rospy.Time.now(),
            #                      'finger_tran', "world")

            rate.sleep()

    def get_robot_vel(self,line_target):
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
        axis_y=-np.array([0, 1, 0])
        axis_z=np.array(vec)
        axis_z_on_y=np.dot(axis_y, axis_z)
        axis_z=axis_z - axis_z_on_y * axis_y
        axis_z=axis_z/np.linalg.norm(axis_z)
        axis_x=np.cross(axis_y, axis_z)

        rot_mat=np.zeros((4, 4))
        rot_mat[:3, 0]=axis_x
        rot_mat[:3, 1]=axis_y
        rot_mat[:3, 2]=axis_z
        rot_mat[3, 3]=1
        q_tf=tf.transformations.quaternion_from_matrix(rot_mat)
        return q_tf

    def get_svm_dir(self,curpos,cut,ok):
        svm_limit=0.1
        offs=0.001
        gain1=50
        gain2=30000
        next_svm_pos=self.desired_end_vec/100
        # self.get_robot_svr(next_svm_pos)
        if self.gamma_dist<cut-offs and ok:
            # self.svm_dir=-self.desired_end_vec*(cut-offs-self.gamma_dist)*(cut-offs-self.gamma_dist)*gain
            # next_svm_pos=(curpos-self.desired_end_vec)*(cut-offs-self.gamma_dist)*gain
            # next_svm_pos=(curpos-self.desired_end_vec)/10
            next_svm_pos=curpos+self.desired_end_vec/10
            # rospy.loginfo("SVR go back "+str(curpos)+" "+str(self.desired_end_vec))
            # next_svm_pos[1:]=0
            self.get_robot_svr(next_svm_pos)
            # self.svm_dir=-(self.world_to_svr_target-self.trans_ee_real)*(cut-offs-self.gamma_dist)*gain1
            self.svm_dir=(self.world_to_svr_target-self.trans_ee_real)*(cut-offs-self.gamma_dist)*(cut-offs-self.gamma_dist)*gain2

            self.svm_dir[1:]=0


        elif self.gamma_dist>=cut+offs and ok:
            # self.svm_dir=self.desired_end_vec*(self.gamma_dist-cut-offs)*(self.gamma_dist-cut-offs)*gain
            # next_svm_pos=-(curpos-self.desired_end_vec)*(self.gamma_dist-cut-offs)*gain
            # next_svm_pos=-(curpos-self.desired_end_vec)/10
            next_svm_pos=curpos-self.desired_end_vec/10
            # rospy.loginfo("SVR go to "+str(curpos)+" "+str(self.desired_end_vec))
            # next_svm_pos[1:]=0
            self.get_robot_svr(next_svm_pos)
            # self.svm_dir=-(self.world_to_svr_target-self.trans_ee_real)*(self.gamma_dist-cut-offs)*gain1
            self.svm_dir=(self.world_to_svr_target-self.trans_ee_real)*(self.gamma_dist-cut-offs)*(self.gamma_dist-cut-offs)*gain2
            self.svm_dir[1:]=0

        else:
            self.svm_dir=[0,0,0]
        # self.svm_dir=[0,0,0]
        # print(np.append(self.svm_dir,0))
        self.svm_dir_tmp=np.dot(tf.transformations.quaternion_matrix(self.trans_svr_ee[1]),np.append(self.svm_dir,0))[:3]
        # print(self.svm_dir)
        # print(self.svm_dir_tmp)
        self.svm_dir_tmp[1:]=0
        self.svm_dir2=self.svm_dir_tmp*self.quat_to_direction(tf.transformations.quaternion_inverse(self.trans_svr_ee[1]))
        svm_dir_norm=np.linalg.norm(self.svm_dir)
        # print(svm_dir_norm)
        if svm_dir_norm>svm_limit:
            self.svm_dir=self.svm_dir/svm_dir_norm*svm_limit

    def get_robot_svr(self,pos):
        # pos=self.trans_end_svr
        # self.svr_target=pos
        self.svr_target=pos
        # rospy.loginfo("SVR target"+str(self.svr_target))
        self.br_ee_svr_target.sendTransform(pos,[0,0,0,1], rospy.Time.now(),'svr_target','SVR')
        # self.listener.waitForTransform('/world', '/svr_target', rospy.Time.now(), rospy.Duration(4.0))
        transformed=False
        # print('svr')
        # while not transformed:
        try:
            trans_world_to_svr_target = np.array(self.listener.lookupTransform(
                'world', 'svr_target', rospy.Time(0)))
            self.world_to_svr_target=np.array(trans_world_to_svr_target[0])
            transformed=True
            if not self.ee_svr_target_logged2:
                rospy.loginfo("ee_svr_target transform received")
                self.ee_svr_target_logged2=True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                a=1
                # continue


    def publish_svr_ee(self,trans_svr_ee):
        self.ee_in_svr.position.x = trans_svr_ee[0][0]
        self.ee_in_svr.position.y = trans_svr_ee[0][1]
        self.ee_in_svr.position.z = trans_svr_ee[0][2]
        self.ee_in_svr.orientation.x = trans_svr_ee[1][0]
        self.ee_in_svr.orientation.y = trans_svr_ee[1][1]
        self.ee_in_svr.orientation.z = trans_svr_ee[1][2]
        self.ee_in_svr.orientation.w = trans_svr_ee[1][3]
        self.RobotPosConvPub.publish(self.ee_in_svr)

    def chatterCallback_RobotEnd(self, data):
        self.end.position.x=data.position.x
        self.end.position.y=data.position.y
        self.end.position.z=data.position.z
        self.end.orientation.x=data.orientation.x
        self.end.orientation.y=data.orientation.y
        self.end.orientation.z=data.orientation.z
        self.end.orientation.w=data.orientation.w
        self.end_received=True

    def chatterCallback_desiredVel(self, data):
        self.desired_end_vec=np.array(
            [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])
        self.desired_end_received=True

    def chatterCallback_Gamma(self, data):
        self.gamma_dist=data.data

    def chatterCallback_Forces(self, data):
        self.forces_vec=0.02*np.array(data.data)+0.98*self.forces_vec
        self.forces_vec[self.forces_vec<0]=0
        self.forces_received=True

    def publish_end_conv(self, data):
        self.Robot_conv.position.x=data[0][0]
        self.Robot_conv.position.y=data[0][1]
        self.Robot_conv.position.z=data[0][2]
        self.Robot_conv.orientation.x=data[1][0]
        self.Robot_conv.orientation.y=data[1][1]
        self.Robot_conv.orientation.z=data[1][2]
        self.Robot_conv.orientation.w=data[1][3]
        self.RobotPosConvPub.publish(self.Robot_conv)

    def publish_end_desired(self, data, data2):
        self.Robot_des.position.x=data[0]
        self.Robot_des.position.y=data[1]
        self.Robot_des.position.z=data[2]
        self.Robot_des.orientation.x=data2[0]
        self.Robot_des.orientation.y=data2[1]
        self.Robot_des.orientation.z=data2[2]
        self.Robot_des.orientation.w=data2[3]
        self.RobotPosDesiredConvertedPub.publish(self.Robot_des)

    # rotate vector v1 by quaternion q1
    def qv_mult(self, q1, v1):
        v_norm=np.linalg.norm(v1)
        v1=tf.transformations.unit_vector(v1)
        q2=list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]*v_norm

    def pubish_on_point_cloud(self, X):
        pointCloud=PointCloud()
        header=Header()
        header.stamp=rospy.Time.now()
        header.frame_id='SVR'
        pointCloud.header=header
        pointCloud.points=[]
        for i in range(len(X)):
            Point_temp=Point32()
            Point_temp.x=X[i][0]
            Point_temp.y=X[i][1]
            Point_temp.z=X[i][2]

            pointCloud.points.append(Point_temp)
        self.CloudPub.publish(pointCloud)

    def load_pointcloud(self):
        self.pointcloud=np.loadtxt(
            "/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/Pointcloud/pointcloud3.txt")

    def publish_arm_and_shoulder(self):
        try:
            # trans_end_hand = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/mocap_hand_filtered', rospy.Time(0))
            trans_end_hand=self.listener.lookupTransform(
                '/SVR', '/mocap_hand_filtered', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to Hand")
        self.Robot_hand.pose.position.x=trans_end_hand[0][0]
        self.Robot_hand.pose.position.y=trans_end_hand[0][1]
        self.Robot_hand.pose.position.z=trans_end_hand[0][2]
        self.Robot_hand.pose.orientation.x=trans_end_hand[1][0]
        self.Robot_hand.pose.orientation.y=trans_end_hand[1][1]
        self.Robot_hand.pose.orientation.z=trans_end_hand[1][2]
        self.Robot_hand.pose.orientation.w=trans_end_hand[1][3]
        self.RobotHandPub.publish(self.Robot_hand)

        try:
            # trans_end_shoulder = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/SVR', rospy.Time(0))
            trans_end_shoulder=self.listener.lookupTransform(
                '/SVR', '/SVR', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to SVR")
        self.Robot_shoulder.pose.position.x=trans_end_shoulder[0][0]
        self.Robot_shoulder.pose.position.y=trans_end_shoulder[0][1]
        self.Robot_shoulder.pose.position.z=trans_end_shoulder[0][2]
        self.Robot_shoulder.pose.orientation.x=trans_end_shoulder[1][0]
        self.Robot_shoulder.pose.orientation.y=trans_end_shoulder[1][1]
        self.Robot_shoulder.pose.orientation.z=trans_end_shoulder[1][2]
        self.Robot_shoulder.pose.orientation.w=trans_end_shoulder[1][3]
        self.RobotShoulderPub.publish(self.Robot_shoulder)

    def init_topics(self):
        self.GammaSub = rospy.Subscriber(
            "/svr/gamma", Float64, self.chatterCallback_Gamma)
        self.RobotPosDesiredSub = rospy.Subscriber(
            "/convert_tf/desired_vel_in_svr", TwistStamped, self.chatterCallback_desiredVel)
        self.GammaSub = rospy.Subscriber(
            "/thumb_forces", Float64MultiArray, self.chatterCallback_Forces)
        self.RobotPosConvPub = rospy.Publisher(
            "/convert_tf/ee_in_svr", Pose, queue_size=3)
        self.RobotCommandPub = rospy.Publisher(
            "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)
        self.CloudPub = rospy.Publisher(
            "/PointCloud/points", PointCloud)
        self.GrabPub = rospy.Publisher(
            "/grab", Int8, queue_size=3)

        # Debug publishers
        self.DebugVelPub = rospy.Publisher(
            "/debug/vel_combined", Pose, queue_size=3)
        self.DebugSvrPub = rospy.Publisher(
            "/debug/svr_combined", Pose, queue_size=3)

    def init_params(self):
        self.desired_end_received = False
        self.Hand_received = False
        self.end = Pose()
        self.ee_in_svr = Pose()
        self.trans_arm = []
        # rx ry rz and vx vy vz (first desired orientation and then desired velocity)
        self.custom_command = Float64MultiArray()
        self.forces_vec=np.array([0,0,0,0,0,0])
        self.svm_dir=np.array([0,0,0])

        self.hand_target=False
        self.custom_command.data = [0, 0, 0, 0, 0, 0]
        self.forces_received = False

        self.target_vec_x=[float(i)/10000 for i in range(1800,1549,-29)]
        self.target_vec_y=[0.05,0.075,0.1,0.125,0.15,0.175,0.2,0.225,0.25]
        self.target_vec_y2=self.target_vec_y[::-1]
        self.end_reached=False

        # Debug
        self.debugVelPose = Pose()
        self.debugSvrPose = Pose()

        # Logging transforms
        self.hand_logged=False
        self.ee_svr_logged=False
        self.svr_ee_logged=False
        self.ee_world_logged=False
        self.ee_real_logged=False
        self.ee_line_target_logged=False
        self.ee_svr_target_logged=False
        self.ee_svr_target_logged2=False
        self.world_svr_logged=False
        self.world_line_logged=False


if __name__ == '__main__':
    convert_tf()
