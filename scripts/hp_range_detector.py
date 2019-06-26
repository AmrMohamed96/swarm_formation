#!/usr/bin/env python
from __future__ import division
import rospy
from math import sin, cos, pi
from std_msgs.msg import Int32, Int32MultiArray, Byte
import numpy as np
import transformations
import time

# range sensor maximum distance in cm
m = 35

# robot position variables
rob1_position = [34,0,0]
rob2_position = [35,0,0]
rob3_position = [33,0,0]
rob4_position = [0,0,0]

# list showing how many robots are in range for robot i
inrange_count = [0,0,0,0]

###############################################################################
# Robot Position Subscriber Functions
###############################################################################
def rob1_position_callback(data):
    global rob1_position
    rob1_position = data.data

def rob2_position_callback(data):
    global rob2_position
    rob2_position = data.data

def rob3_position_callback(data):
    global rob3_position
    rob3_position = data.data

def rob4_position_callback(data):
    global rob4_position
    rob4_position = data.data

###############################################################################
# Re-Running the detection algorithm callback
###############################################################################
def rerun_range_sensors_callback(data):
    if data.data == 1:
        check_in_range_rob1()
        check_in_range_rob2()
        check_in_range_rob3()
        check_in_range_rob4()
    else:
        return

###############################################################################
# Setting a leader based on data, and checking if it needs motion
###############################################################################
def set_and_check_leader():
    """
    Function loops over the inrange_count list, checks how sees the most
    robots in the system and assigns them as a leader

    If 2 robots have the same number, the first one in the list will be assigned
    as a leader

    EDITING REQUIRED ***********************************************************
    """
    # check which robot is the leader
    desired_leader = 0
    max = 0
    for i in range( len(inrange_count) ):
        if ( inrange_count[i] >= max ):
            max = inrange_count[i]
            desired_leader = i

    # check if a formation is possible given the current situation

    # assign a next goal for leader if needed

    # set the leader
    setLead = rospy.Publisher('set_new_leader', Byte, queue_size=1)
    time.sleep(5)
    setLead.publish(desired_leader+1)
    rospy.loginfo('Leader will be Robot {} as it sees {} robots'.format(desired_leader + 1, inrange_count[desired_leader]))

###############################################################################
# Range Detector Simulator Functions
###############################################################################
def check_points_in_range(current_rob_pos, other_rob_pos):
    # calculate bottom right corner of the bounding rectangle
    rx, ry, rth = current_rob_pos
    zx = round( rx - ( m * sin(rth) ), 3)
    zy = round( ry + ( m * cos(rth) ), 3)

    # calculating the transformation matrix of the new frame at that vertex
    alpha, beta, gamma = 0, 0, ( -( pi/2 - rth) )
    origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    Rx = transformations.rotation_matrix(alpha, xaxis)
    Ry = transformations.rotation_matrix(beta, yaxis)
    Rz = transformations.rotation_matrix(gamma, zaxis)
    T = transformations.translation_matrix([zx, zy, 0])
    R = transformations.concatenate_matrices(Rx, Ry, Rz)
    Z = transformations.shear_matrix(beta, xaxis, origin, zaxis)
    S = transformations.scale_matrix(1, origin)
    M = transformations.concatenate_matrices(T, R, Z, S)
    M = np.linalg.inv(M)

    # calculating the new point in that frame
    other_x = other_rob_pos[0]
    other_y = other_rob_pos[1]
    other_z = 0
    other_h = 1
    other_point_homogenous = np.array([other_x, other_y, other_z, other_h])
    new_point = np.dot(M,other_point_homogenous)
    new_point = np.round(new_point, 3)

    # checking that the point lies within the boundaries
    px, py, pz, ph = new_point
    if px <= (2 * m) and px >=0 and py <= m and py >=0:
        return 1
    else:
        return 0

def check_in_range_rob1():
    global inrange_count
    inrange_count[0] = 0
    inrange_count[0] += check_points_in_range(rob1_position, rob2_position)
    inrange_count[0] += check_points_in_range(rob1_position, rob3_position)
    inrange_count[0] += check_points_in_range(rob1_position, rob4_position)

def check_in_range_rob2():
    global inrange_count
    inrange_count[1] = 0
    inrange_count[1] += check_points_in_range(rob2_position, rob1_position)
    inrange_count[1] += check_points_in_range(rob2_position, rob3_position)
    inrange_count[1] += check_points_in_range(rob2_position, rob4_position)

def check_in_range_rob3():
    global inrange_count
    inrange_count[2] = 0
    inrange_count[2] += check_points_in_range(rob3_position, rob1_position)
    inrange_count[2] += check_points_in_range(rob3_position, rob2_position)
    inrange_count[2] += check_points_in_range(rob3_position, rob4_position)

def check_in_range_rob4():
    global inrange_count
    inrange_count[3] = 0
    inrange_count[3] += check_points_in_range(rob4_position, rob1_position)
    inrange_count[3] += check_points_in_range(rob4_position, rob2_position)
    inrange_count[3] += check_points_in_range(rob4_position, rob3_position)


if __name__ == '__main__':
    try:
        # initialize ros node
        rospy.init_node('hp_range_detector')
        rospy.loginfo('%s started' % rospy.get_name())

        # initialize subscribers to positions
        rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, rob1_position_callback)
        rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, rob2_position_callback)
        rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, rob3_position_callback)
        rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, rob4_position_callback)
        rospy.Subscriber('rerun_range_sensors', Byte, rerun_range_sensors_callback)

        # initialize publisher
        rangePub = rospy.Publisher('inrange_count', Int32MultiArray, queue_size=10)

        # run the checking algorithm once
        check_in_range_rob1()
        check_in_range_rob2()
        check_in_range_rob3()
        check_in_range_rob4()
        set_and_check_leader()

        # stay in a loop publishing until another run is issues via the subscriber
        try:
            while not rospy.is_shutdown():
                rangePub.publish( Int32MultiArray(data=inrange_count) )
        except KeyboardInterrupt:
            rospy.loginfo('Keyboard Interrupt Exception')

    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
