#!/usr/bin/env python
import rospy 
from svr_grad_ros.srv import *
import re



finger_tip_position_0=[0.0399453,0.0626116,0.123999]
finger_tip_position_1=[0.0399453,0.00809732,0.127429]
finger_tip_position_2=[0.0399453,-0.0464786,0.10541]
finger_tip_position_3=[0.0350638,0.156066,-0.0578007]


print(type(finger_tip_position_0))

rospy.wait_for_service("/compute_ik_joint_angles")
try:
    joint_config = rospy.ServiceProxy('/compute_ik_joint_angles',compute_joint_angels)
    # res=joint_config('finger_tip_position_0','finger_tip_position_1','finger_tip_position_2','finger_tip_position_3')
    print((finger_tip_position_0,finger_tip_position_1,finger_tip_position_2,finger_tip_position_3))
    res=joint_config(finger_tip_position_0,finger_tip_position_1,finger_tip_position_2,finger_tip_position_3)
    # joint_config(1)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

print(res)