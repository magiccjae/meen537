#system level imports
import sys, os
from collections import deque
import numpy as np
import scipy.io as sio
#!/usr/local/bin/python

from copy import deepcopy
from threading import RLock, Timer
import time
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('me_537_lab')
    limb = RadBaxterLimb('left')

    raw_input("go to position 1 using the wrist button on baxter ...")
    pose = limb.get_kdl_forward_position_kinematics()
    R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
    position = pose[0:3]
    print "joint angles are: \t", limb.get_joint_angles()
    print "end effector position is: \t", position
    print "end effector orientation is: \t", R, "\n\n\n"


    raw_input("go to position 2 using the wrist button on baxter ...")
    pose = limb.get_kdl_forward_position_kinematics()
    R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
    position = pose[0:3]
    print "joint angles are: \t", limb.get_joint_angles()
    print "end effector position is: \t", position
    print "end effector orientation is: \t", R, "\n\n\n"

    raw_input("go to position 3 using the wrist button on baxter ...")
    pose = limb.get_kdl_forward_position_kinematics()
    R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
    position = pose[0:3]
    print "joint angles are: \t", limb.get_joint_angles()
    print "end effector position is: \t", position
    print "end effector orientation is: \t", R, "\n\n\n"

    raw_input("go to position 4 using the wrist button on baxter ...")
    pose = limb.get_kdl_forward_position_kinematics()
    R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
    position = pose[0:3]
    print "joint angles are: \t", limb.get_joint_angles()
    print "end effector position is: \t", position
    print "end effector orientation is: \t", R, "\n\n\n"


    # #modify this as you want
    # joint_command = [0, 0, 0, 0, 0, 0, 0]

    # while not rospy.is_shutdown():
    #     control_rate = rospy.Rate(500)
    #     limb.set_joint_positions_mod(joint_command)
    #     control_rate.sleep()
