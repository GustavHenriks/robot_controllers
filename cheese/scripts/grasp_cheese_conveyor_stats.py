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
from std_msgs.msg import Header, Float64MultiArray, Float64, Int8, String, Int32
from sensor_msgs.msg import PointCloud
import time
from nav_msgs.msg import Path

class grasp_chesse_conveyor_stats():
    def __init__(self):

        freq = 200
        rospy.init_node('grasp_cheese_conveyor_stats', anonymous=True)

        self.init_broadcasts()
        self.init_topics()
        self.init_params()

        rate = rospy.Rate(freq)
        start = time.time()

    
        print("Starting loop")
        rot_mat=np.ones(4)
        bun_counter=0
        while not rospy.is_shutdown():

            # Get End effector of robot
            try:
                trans_ee_real = self.listener.lookupTransform(
                    'world', 'iiwa_link_ee', rospy.Time(0))
                if not self.ee_svr_logged:
                    rospy.loginfo("ee_real transform received")
                    self.ee_svr_logged = True
                    self.target_position=np.array([0.0,-0.42,0.736])
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
            # try:
            #     # common_time = self.listener.getLatestCommonTime(
            #     # '/palm_link', '/target') 
            #     trans_target_raw = self.listener.lookupTransform(
            #         'mocap_world', 'mocap_test_object', rospy.Time(0))
            #     if not self.trans_target_logged:
            #         rospy.loginfo("target transform received")
            #         self.trans_target_logged = True
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue      
            # if not self.target_read:
            #     trans_target=trans_target_raw
            #     trans_target_raw_rotated=tf.transformations.quaternion_multiply(trans_target_raw[1],trans_world[1])
                # rospy.loginfo("Target position and orientation locked")                
                # self.target_read=True
            # trans_target=trans_target_raw
            # trans_target_raw_rotated=tf.transformations.quaternion_multiply(trans_target_raw[1],trans_world[1])
            # print(tf.transformations.quaternion_matrix(trans_target[1]),np.append(np.array(trans_world[0]),1))
            if self.start_motion:
                try: 
                    trans_target_position = self.listener.lookupTransform(
                        'world', 'Center', rospy.Time(0))
                    if not self.trans_target_position_logged:
                        rospy.loginfo("Center transform received")
                        self.trans_target_position_logged = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue 
                self.trans_target_position=np.array(trans_target_position[0])+np.array([-0.2,0.05,0])

            ## Use the orientaion given by PCA
            # offset=np.dot(tf.transformations.quaternion_matrix(trans_target[1]),np.array([-0.03,-0.04,0,1]))[:-1] #-0.03,-0.018 worked ok 20191022
            # trans_world_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_world[0]),1))
            # trans_target_rotated=np.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(trans_world[1])),np.append(np.array(trans_target[0])+offset,1))
            # self.trans_target=trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0,0,0.14])+self.salad_offset

            # Testing grasping off chesse
            self.trans_target=np.array([self.trans_target_position[0],self.trans_target_position[1],self.trans_target_test[2]])
            self.attack_position=self.trans_target+np.array([-0.05,0,0.2])
            # self.attack_position=self.trans_target
            if not self.target_position_initialized and self.start_motion==True:
                self.target_position=self.attack_position
                self.target_position_initialized=True
                self.start_motion=False
            

                
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
                # self.desired_orientation_cheese = [ 1.4778077, 1.4692665, -1.0090587 ] 
                # self.desired_orientation_cheese = [0, np.deg2rad(117), 0]  # EE aligned with conveyor belt
                self.desired_orientation_cheese = [0.2081947, 2.0178752, -0.4424049] # Tweezer aligned with conveyor belt
                self.custom_command.data[0:3] = self.desired_orientation_cheese
                self.custom_command.data[3:6] = [0, 0, 0.03]
                t_start = time.time()
                counter = 0
                while counter < 50:
                    self.RobotCommandPub.publish(self.custom_command)
                    self.GripperPub.publish(self.gripperMsg)
                    counter = counter+1
                    rate.sleep()
                self.go_to_init_possition2 = False
                raw_input('Waiting to start movement')               
            # self.desired_orientation=self.quat_to_direction(trans_target[1])
            # self.desired_orientation=self.quat_to_direction(trans_target_raw_rotated)
            # self.desired_orientation= [0, np.deg2rad(180),0]  
            self.current_position = self.trans_ee_real
            direction_to_target = (self.target_position - self.current_position)
            distance_to_target = np.linalg.norm(direction_to_target)
            self.desired_velocity = self.robot_gain*(direction_to_target)
            desired_velocity_norm = np.linalg.norm(self.desired_velocity)
            self.desired_velocity[2]=self.desired_velocity[2]*self.z_gain
            if desired_velocity_norm>self.max_vel:
                self.desired_velocity=self.desired_velocity/desired_velocity_norm*self.max_vel
                
            # self.custom_command.data[0:3] = self.desired_orientation
            self.custom_command.data[0:3] = self.desired_orientation_cheese
            self.custom_command.data[3:6] = self.desired_velocity
            self.RobotCommandPub.publish(self.custom_command)
            # self.br_ee_target.sendTransform(trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([-0.025,0.009,0.0]), [0,0,0,1], rospy.Time.now(
            # ), 'target_position_rf', "world")
            # self.br_ee_target.sendTransform(trans_target_rotated[:-1]-trans_world_rotated[:-1]+np.array([0,0,0.19]), trans_target_raw_rotated, rospy.Time.now(
            # ), 'target_position_rf', "world")           
            # self.br_ee_target_dbg.sendTransform(self.attack_position, [0,0,0,1], rospy.Time.now(
            # ), 'attack_position_rf', "world")            
            rospy.loginfo_throttle(1, "Max speed: "+str(np.linalg.norm(self.desired_velocity)))
            rospy.loginfo_throttle(1, "Z_gain: "+str(np.linalg.norm(self.z_gain)))

            # rospy.loginfo_throttle(1, "Target: "+str(self.target_position))

            # Go to initial position to grasp cheese
            if distance_to_target<0.03 and self.attack_reached==False and self.target_position_initialized==True:
                rospy.loginfo_throttle(1, ['Loop 1'])
                self.target_position=self.trans_target+np.array([-0.08,0,0.01])
                if self.counter0<100:
                    self.counter0+=1
                else:
                    self.attack_reached=True
                    self.max_vel=0.15
                    self.target_position=self.trans_target
                    self.z_gain=2

            # Go to grasping position and grasp object
            elif distance_to_target<0.02 and self.attack_reached==True and self.target_reached==False:
                self.z_gain=1
                rospy.loginfo_throttle(1, ['Loop 2'])
                self.gripperMsg.rPR =self.gripper_closed
                self.target_position=self.trans_target
                if self.counter1 < 100:
                    self.GripperPub.publish(self.gripperMsg)
                    self.counter1 = self.counter1+1
                elif self.counter1>=100:
                    self.cheesemsg.data=1
                    self.CheesePub.publish(self.cheesemsg)

                    rospy.loginfo_throttle(1, ['Target reached'])
                    self.target_reached = True     
                print(self.counter1)

            # Go away from object
            if self.target_reached==True and self.pick_up_1==False:
                self.max_vel=0.6
                rospy.loginfo_throttle(1, ['Loop 3'])                
                self.target_position=self.trans_target+np.array([0,0,0.25])
                rospy.loginfo_throttle(1, ['Picked up object'])
                self.pick_up_1=True
            
            # Go to inital position to drop cheese
            elif self.pick_up_1==True and distance_to_target<0.05 and self.pick_up_2==False:
                rospy.loginfo_throttle(1, ['Loop 4'])                
                self.target_position=self.drop_position+np.array([0.0,0.05,0.15])
                self.desired_orientation_cheese = [0, np.deg2rad(116), 0] #Perpendicular drop off
                self.desired_orientation_cheese = [ -0.6030955, 2.1452044, 0.4424738 ] #tilted drop off
                self.pick_up_2=True

            # Go to initial position 2 to drop cheese
            elif self.pick_up_2==True and distance_to_target<0.05 and self.pick_up_3==False:
                rospy.loginfo_throttle(1, ['Loop 5'])                
                self.target_position=self.drop_position+np.array([0.06,0,0]) #Perpendicular drop off
                self.target_position=self.drop_position+np.array([0.04,0.04,0]) #Tilted drop off
                self.pick_up_3=True    

            # Go to dropping position and drop cheese

            elif self.pick_up_3==True and distance_to_target<0.03 and self.pick_up_4==False:
                rospy.loginfo_throttle(1, ['Loop 6'])                    
                self.target_position=self.drop_position+np.array([-0.02,0,-0.02]) #Perpendicular drop off
                self.target_position=self.drop_position+np.array([-0.01,-0.01,-0.02]) #Tilted drop off
                self.pick_up_4=True
            
            elif self.pick_up_4==True and distance_to_target<0.031 and self.pick_up_5==False:
                rospy.loginfo_throttle(1, ['Loop 7']) 
                self.target_position=self.drop_position+np.array([-0.15,-0.15,0.2])          
                # self.desired_orientation_cheese = [0, np.deg2rad(140), 0] # Perpendicular drop off
                self.desired_orientation_cheese = [-0.7468389, 2.3450905, 0.2261812] # Tilted drop off 
                self.gripperMsg.rPR =self.gripper_open2
                if self.counter2 < 50:
                    self.GripperPub.publish(self.gripperMsg)
                    self.counter2 = self.counter2+1
                elif self.counter2>=50:
                    # self.cheesemsg.data=0
                    # self.CheesePub.publish(self.cheesemsg)                    
                    rospy.loginfo_throttle(1, ['Target reached'])         
                    self.pick_up_5=True

            elif self.pick_up_5==True and self.start_motion==True:
                # self.drop_position[2]+=0.01                
                self.reset_bool()
                # bun_counter+=1
                # if bun_counter==3:
                #     self.gripper_open=120
                #     self.gripper_closed=200
            # rospy.loginfo_throttle(1, [self.grasped==0])
            # rospy.loginfo_throttle(1, ['Dist to target '+str(distance_to_target)])            
            # rospy.loginfo_throttle(1, ['Target pos'+ str(self.target_position)])

            rate.sleep()


    def reset_bool(self):
        self.trans_target=self.trans_target_orig
        self.attack_reached=False
        self.target_reached=False
        self.target_position_initialized=False
        self.allegroMsg = String()
        self.pick_up_1=False
        self.pick_up_2=False
        self.pick_up_3=False
        self.pick_up_4=False
        self.pick_up_5=False
        self.counter0=0
        self.counter1=0
        self.counter2=0
        self.gripper_closed=219
        self.gripper_open=150
        self.gripperMsg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripperMsg.rACT=1
        self.gripperMsg.rGTO=1
        self.gripperMsg.rATR=0
        self.gripperMsg.rPR =self.gripper_open
        self.gripperMsg.rSP =255
        self.gripperMsg.rFR =150
        self.desired_orientation_cheese = [0.2081947, 2.0178752, -0.4424049]

    def chatterCallback_objectInFrame(self,msg):
        if msg.data==1:
            self.start_motion=True



    def init_broadcasts(self):
        self.listener = tf.TransformListener()
        self.br_robot_base_fixed = tf.TransformBroadcaster()
        self.br_ee_target = tf.TransformBroadcaster()
        self.br_ee_target_dbg = tf.TransformBroadcaster()

    def init_topics(self):
        self.RobotCommandPub = rospy.Publisher(
            "/iiwa/CustomControllers/command", Float64MultiArray, queue_size=3)    
        self.GripperPub = rospy.Publisher(
            "/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=3
        )
        self.CheesePub = rospy.Publisher(
            "/conveyor/cheese", Int32, queue_size=3
        )
        self.ObjectInFrameSub = rospy.Subscriber(
            "/camera/object_in_frame", Int32, self.chatterCallback_objectInFrame
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
        self.trans_target_position_logged = False

        # End effector transform
        self.ee_svr_logged = False
        self.trans_target_logged = False
        self.trans_world_logged = False

        # Robot parameters
        self.max_vel = 0.6
        self.robot_gain = 7

        # Grasping
        self.grasped = 0
        self.allegroMsg = String()
        self.trans_target_orig=np.array([0.1,-0.62,0.248]) #0.236 for one plate only, 0.245 for 5 plates
        self.trans_target_test=self.trans_target_orig
        self.trans_target_position=self.trans_target_orig
        # self.drop_position = np.array([0.50,-0.2,0.25]) # Without burger
        self.drop_position = np.array([0.50,-0.2,0.27]) # With patty
        self.drop_position = np.array([0.50,-0.2,0.28]) # With patty+salad
        self.pick_up_1=False
        self.pick_up_2=False
        self.pick_up_3=False
        self.pick_up_4=False
        self.pick_up_5=False
        self.counter0=0
        self.counter1=0
        self.counter2=0
        self.gripper_closed=219 # ORiginal
        # self.gripper_closed=200 # Bun top
        self.gripper_open=150 # Original
        # self.gripper_open=120 # Bun top
        self.gripper_open2=140
        self.gripperMsg = outputMsg.Robotiq2FGripper_robot_output()
        self.gripperMsg.rACT=1
        self.gripperMsg.rGTO=1
        self.gripperMsg.rATR=0
        self.gripperMsg.rPR =self.gripper_open
        self.gripperMsg.rSP =255
        self.gripperMsg.rFR =150
        self.z_gain=1

        # Cheese communication
        self.cheesemsg=Int32()
        self.start_motion=False
        self.burger_offset=0
        



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
    grasp_chesse_conveyor_stats()
