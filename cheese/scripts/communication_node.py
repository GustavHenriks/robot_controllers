#!/usr/bin/env python
## Used to pick up objects from the top
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Int32
import time

class communication_node():
    def __init__(self):

        freq = 200
        rospy.init_node('communication_node', anonymous=True)

        # self.init_broadcasts()
        self.init_topics()
        self.init_params()

        rate = rospy.Rate(freq)
        rate2 = rospy.Rate(1)
        print("Establishing connection with conveyor belt")
        rate2.sleep()
        count=0
        print("Starting loop")
        time_init=False
        pub_sent=False
        while not rospy.is_shutdown():
            if self.Start_conveyor==1 and self.published_go==False:
                rospy.loginfo_throttle(1, ['Publishing go'])     
                self.conveyormodedata.data=1
                self.ConveyorModePub.publish(self.conveyormodedata)
                self.published_go=True
                self.published_stop=False
            if self.Start_conveyor==0 and self.published_stop==False:
                if time_init==False:
                    t1=time.time()
                    time_init=True
                rospy.loginfo_throttle(1, ['Publishing stop'])     
                self.conveyormodedata.data=0
                if time.time()-t1>.3 and pub_sent==False:
                    rospy.loginfo_throttle(1, ['Stopping Conveyor belt']) 
                    self.ConveyorModePub.publish(self.conveyormodedata)   
                    pub_sent=True
                if time.time()-t1>5:
                    rospy.loginfo_throttle(1, ['Target ready'])     
                    self.published_go=False
                    self.published_stop=True
                    self.robotmsg.data=1
                    time_init=False
                    pub_sent=False
                    self.StartRobotPub.publish(self.robotmsg)   
            rate.sleep()

        ## Automatic testing
        # while not rospy.is_shutdown() and self.run_ros==True:
        #     if count<200:
        #         if count==0:
        #             print('Publishing go')
        #             self.conveyormodedata.data=1
        #             self.ConveyorModePub.publish(self.conveyormodedata)
        #         count+=1
        #     elif count<600:
        #         if count==200:
        #             print('Publishing stop')
        #             self.conveyormodedata.data=0
        #             self.ConveyorModePub.publish(self.conveyormodedata)
        #         count+=1
        #     else:
        #         self.run_ros=False
        #     rate.sleep()        


    def init_broadcasts(self):
        self.listener = tf.TransformListener()
        self.br_robot_base_fixed = tf.TransformBroadcaster()
        self.br_ee_target = tf.TransformBroadcaster()
        self.br_ee_target_dbg = tf.TransformBroadcaster()

    def chatterCallback_Cheese(self,msg):
        print('received')
        self.Start_conveyor=msg.data
        print(msg.data)


    def init_topics(self):
        self.ConveyorModePub = rospy.Publisher(
            "/conveyor_belt/desired_mode", Int32, queue_size=3)        
        self.ConveyorModeSub = rospy.Subscriber("/conveyor/cheese", Int32, self.chatterCallback_Cheese)
        self.StartRobotPub = rospy.Publisher(
            "/camera/object_in_frame", Int32, queue_size=3)        
        # self.GrabPub = rospy.Publisher(
        #      "/grab", Int8, queue_size=3)
        # self.GraspedSub = rospy.Subscriber(
        #     "/grasped", Int8, self.chatterCallback_Grasped)
        # self.SaladSub = rospy.Subscriber(
        #     "/salad", Int8, self.chatterCallback_Salad)
        # self.AllegroPub = rospy.Publisher(
        #     "/allegroHand_1/lib_cmd", String, queue_size=3)
        # self.GripperPub = rospy.Publisher(
        #     "/Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=3
        # )


    def init_params(self):
        # Conveyor belt
        self.conveyormodedata=Int32()
        self.Start_conveyor=0
        self.run_ros=True
        self.published_go=False
        self.published_stop=True
        self.robotmsg=Int32()

if __name__ == '__main__':
    communication_node()
