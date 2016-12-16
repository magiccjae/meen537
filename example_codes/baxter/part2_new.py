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
    control_rate = rospy.Rate(500)

    #modify this variable to change the number of times it runs
    num_times = 10

    raw_input("move to your desired position and push enter ...")
    joint_command = limb.get_joint_angles()
    joint_command_start = np.array([0, 0, 0, 0, 0, 0, 0])



    for i in xrange(num_times):

        step = 1
        while step < 2500:
            limb.set_joint_positions_mod(joint_command_start)
            control_rate.sleep()
            step = step + 1

        step = 1
        while step < 2500:
            control_rate = rospy.Rate(500)
            limb.set_joint_positions_mod(joint_command)
            control_rate.sleep()
            step = step + 1

        pose = limb.get_kdl_forward_position_kinematics()
        R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
        position = pose[0:3]
        print "for test number "+str(i).zfill(2)+":"
        print "joint angles are: \t", limb.get_joint_angles()
        print "end effector position is: \t", position
        print "end effector orientation is: \t", R, "\n\n\n"
        
        raw_input('measure the vertical direction and then push enter when ready for the next cycle ...')
