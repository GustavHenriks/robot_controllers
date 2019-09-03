#!/usr/bin/env python
import rospy
from svr_grad_ros.srv import *
import re
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
from tf.transformations import *


class test_service():

    def __init__(self):
        self.finger_tip_position_0 = np.array([0.0799453, 0.0626116, 0.093999])
        self.finger_tip_position_1 = np.array([0.0399453, 0.00809732, 0.127429])
        self.finger_tip_position_2 = np.array([0.0399453, -0.0464786, 0.10541])
        # self.finger_tip_position_3 = np.array([0.0350638, 0.156066, -0.0588007])
        self.finger_tip_position_3 = np.array([0.0998, 0.03066, -0.0245407]) #X: palm out,Y: going left towards thumb, Z: thumb going towards fingers
        # self.finger_tip_position_3 = np.array([0.0350638, 0.126066, -0.0388007])
        # self.finger_tip_position_3 = np.array([0.070638, 0.056066, 0.1188007])
        rospy.init_node('test_service', anonymous=True)
        self.pub = rospy.Publisher(
            "/allegroHand_0/joint_cmd", JointState, queue_size=3)
        self.sub = rospy.Subscriber("/Finger/Pos/Thumb",Pose,self.chatterCallback_finger)
        self.sub = rospy.Subscriber("/allegroHand_0/joint_states",JointState,self.chatterCallback_allero)
        self.listener = tf.TransformListener()
        self.get_transforms()
        self.desired = JointState()
        self.br_ee_finger = tf.TransformBroadcaster()
        self.br_base_hand = tf.TransformBroadcaster()
        self.orientaion=rotation_matrix(np.pi,(0,0,1))
        self.orientaion2=rotation_matrix(np.pi/2,(0,1,0))
        self.orientaion=np.dot(self.orientaion2,self.orientaion)
        self.finger_received = False
        # self.send_command()
        # self.main_loop_one()
        # self.main_loop_two()
        self.main_loop_three()

    def send_command(self):
        i=0
        while not rospy.is_shutdown():
            rospy.wait_for_service("/compute_ik_joint_angles")
            try:
                joint_config=rospy.ServiceProxy(
                    '/compute_ik_joint_angles', compute_joint_angels)
                # res=joint_config('finger_tip_position_0','finger_tip_position_1','finger_tip_position_2','finger_tip_position_3')
                print((self.finger_tip_position_0, self.finger_tip_position_1,
                    self.finger_tip_position_2, self.finger_tip_position_3))
                # print(np.linalg.norm(finger_tip_position_0),np.linalg.norm(finger_tip_position_1),np.linalg.norm(finger_tip_position_2),np.linalg.norm(finger_tip_position_3))
                res=joint_config(self.finger_tip_position_0, self.finger_tip_position_1,
                                self.finger_tip_position_2, self.finger_tip_position_3)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            if res.found_solution==1:
                self.desired.position=res.joint_angles
                j=0
                # print(desired)
                while j<50:
                    self.pub.publish(self.desired)
                    j=j+1
                    rospy.sleep(0.001)
            else:
                print("No solution found.")
                # try:
                #     ee_to_finger = self.listener.lookupTransform(
                #         'iiwa_link_ee', 'Finger 3', rospy.Time(0))
                # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #     continue
            print('Broadcasting')
            print(self.orientaion)
            print(np.append(self.finger_tip_position_3,1))
            rot_it=rotation_matrix(np.pi,(1,0,0))
            rot_it2=rotation_matrix(-np.pi/2 ,(0,1,0))
            thumb_tmp=np.append(self.thumb_position,1)
            thumb_position=np.dot(rot_it2,np.dot(rot_it,thumb_tmp))
            self.br_ee_finger.sendTransform(thumb_position, [0,0,0,1], rospy.Time.now(),
                                    'finger_test', "iiwa_link_ee")

    def main_loop_one(self):
        self.f30o=self.finger_tip_position_3[0]
        self.f31o=self.finger_tip_position_3[1]
        self.f32o=self.finger_tip_position_3[2]
        i=0
        while not rospy.is_shutdown():
            # finger_tip_position_3[0]=f30o+0.005*np.sin(-np.deg2rad(i))
            # finger_tip_position_3[1]=f31o+0.005*np.sin(-np.deg2rad(i))
            self.finger_tip_position_3[2]=self.f32o+0.01*np.cos(-np.deg2rad(i))
            i=i+5
            rospy.wait_for_service("/compute_ik_joint_angles")
            try:
                joint_config=rospy.ServiceProxy(
                    '/compute_ik_joint_angles', compute_joint_angels)
                # res=joint_config('finger_tip_position_0','finger_tip_position_1','finger_tip_position_2','finger_tip_position_3')
                print((self.finger_tip_position_0, self.finger_tip_position_1,
                    self.finger_tip_position_2, self.finger_tip_position_3))
                # print(np.linalg.norm(finger_tip_position_0),np.linalg.norm(finger_tip_position_1),np.linalg.norm(finger_tip_position_2),np.linalg.norm(finger_tip_position_3))
                res=joint_config(self.finger_tip_position_0, self.finger_tip_position_1,
                                self.finger_tip_position_2, self.finger_tip_position_3)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            if res.found_solution==1:
                self.desired.position=res.joint_angles
                j=0
                # print(desired)
                while j<50:
                    self.pub.publish(self.desired)
                    j=j+1
                    rospy.sleep(0.001)
            else:
                print("No solution found.")
                # try:
                #     ee_to_finger = self.listener.lookupTransform(
                #         'iiwa_link_ee', 'Finger 3', rospy.Time(0))
                # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #     continue
            print('Broadcasting')
            print(self.orientaion)
            print(np.append(self.finger_tip_position_3,1))
            rotated_pos=np.dot(np.append(self.finger_tip_position_3,1),self.orientaion)
            rotated_pos=rotated_pos[:3]*[-1,-1,-1]
            self.br_ee_finger.sendTransform(rotated_pos, [0,0,0,1], rospy.Time.now(),
                                    'finger_test', "iiwa_link_ee")
    def main_loop_two(self):
        i=0
        while not rospy.is_shutdown():
            rospy.wait_for_service("/compute_ik_joint_angles")
            try:
                joint_config=rospy.ServiceProxy(
                    '/compute_ik_joint_angles', compute_joint_angels)
                # res=joint_config('finger_tip_position_0','finger_tip_position_1','finger_tip_position_2','finger_tip_position_3')
                # print((finger_tip_position_0, finger_tip_position_1,
                        # finger_tip_position_2, finger_tip_position_3))
                # print(np.linalg.norm(finger_tip_position_0),np.linalg.norm(finger_tip_position_1),np.linalg.norm(finger_tip_position_2),np.linalg.norm(finger_tip_position_3))
                res=joint_config(self.finger_tip_position_0, self.finger_tip_position_1,
                                self.finger_tip_position_2, self.finger_tip_position_3)
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            if res.found_solution==1:
                    self.desired.position=res.joint_angles
                    i=0
                    # print(desired)
                    while i<50:
                        self.pub.publish(desired)
                        i=i+1
                        rospy.sleep(0.01)
            else:
                print("No solution found.")
            print('Broadcasting')
            print(self.orientaion)
            print(np.append(self.finger_tip_position_3,1))
            rotated_pos=np.dot(np.append(self.finger_tip_position_3,1),self.orientaion)
            rotated_pos=rotated_pos[:3]*[-1,-1,-1]
            self.br_ee_finger.sendTransform(rotated_pos, [0,0,0,1], rospy.Time.now(),
                                    'finger_test', "iiwa_link_ee")
            print(res.joint_angles)

            quaternion_matrix(quaternion_about_axis(180,(0,1,0)))
    
    def main_loop_three(self):
        while not self.finger_received:
            rospy.sleep(0.01)
        while not rospy.is_shutdown():
            rot_quat=quaternion_from_matrix(rotation_matrix(np.pi/2,(0,1,0)))
            rot_quat2=quaternion_from_matrix(rotation_matrix(np.pi,(0,0,1)))
            ori2=quaternion_multiply(self.ee[1],rot_quat)
            ori=quaternion_multiply(ori2,rot_quat2)
            rot_it=rotation_matrix(np.pi,(1,0,0))
            rot_it2=rotation_matrix(-np.pi/2 ,(0,1,0))
            thumb_tmp=np.append(self.thumb_position,1)
            thumb_position=np.dot(rot_it2,np.dot(rot_it,thumb_tmp))
            # print(thumb_position[:3]*[1,-1,1]-[ 0.004511  ,  0.00401849,  0.10461377])
            # rotated_pos=np.dot(np.append(self.thumb_position,1))
            # rotated_pos=rotated_pos[:3]*[-1,-1,-1]
            # self.br_base_hand.sendTransform(self.ee[0],quaternion_from_matrix(self.orientaion),rospy.Time.now(),
            #                      'hand_frame','world')
            self.br_base_hand.sendTransform(self.ee[0],ori,rospy.Time.now(),
                                 'hand_frame','world')
            self.br_ee_finger.sendTransform(thumb_position[:3], [0,0,0,1], rospy.Time.now(),
                                   'finger_test', "hand_frame")
            try:
                joint_config=rospy.ServiceProxy(
                    '/compute_ik_joint_angles', compute_joint_angels)
                res=joint_config(self.finger_tip_position_0, self.finger_tip_position_1,
                                self.finger_tip_position_2,thumb_position[:3]*[1,-1,1]-[ 0.004511  ,  0.00401849,  0.10461377])
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            rospy.sleep(0.01)
            print(res.joint_angles[12:16])
            print(self.thumb_joints)


    def get_transforms(self):
        received=False
        while not received:    
            try:
                self.ee = self.listener.lookupTransform(
                        '/world', '/iiwa_link_ee', rospy.Time(0))
                received=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('transform not received')
            # continue    

    def chatterCallback_finger(self,msg):
        self.thumb_position=np.array([msg.position.x, msg.position.y, msg.position.z])
        self.finger_received=True

    def chatterCallback_allero(self,msg):
        self.thumb_joints=np.array(msg.position[12:16])
        self.finger_received=True

if __name__ == '__main__':
    test_service()
