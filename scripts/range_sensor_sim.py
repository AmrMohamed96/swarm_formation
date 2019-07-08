#!/usr/bin/env python
from __future__ import division
import rospy
from math import sin, cos, pi
from std_msgs.msg import Int32, Int32MultiArray, Byte, String
import numpy as np
import transformations
import time

"""
Range Sensors Simulation Node

FUNCTION: Takes robot positions and orientation, and outputs how many robots are in range
          for each robot.

INPUT : Robot Positions in cm.
OUTPUT: /in_range_count topic which is a list indicating how many robots
        each robot sees.

PARAMS: /grid_dimension : Dimension of each square of the grid.
        /grid_blocks    : No of squares in the grid "Assuming a square grid".
        /visibility_blocks_range : No of blocks ahead of the robot that it can see.
"""

# m is the range sensor maximum distance in cm
grid_dimension = 0
grid_blocks = 0
visible_blocks = 0
m = 0

# robot position variables
rob1_position = [0,0,0]
rob2_position = [0,0,0]
rob3_position = [0,0,0]
rob4_position = [0,0,0]

# list showing how many robots are in range for robot i
inrange_count = [0,0,0,0]

# shape parameters and robot position variables
current_shape = ''
current_shape_length = -1
current_leader_pos = [-1,-1,-1]

# delay in seconds to account for any possible latency in communication
system_latency = 5

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
        check_leader()
    else:
        return
        
def check_leader():
    # check which robot is the leader
    desired_leader = 0
    max = 0
    for i in range( len(inrange_count) ):
        if ( inrange_count[i] > max ):
            max = inrange_count[i]
            desired_leader = i

    # Notify the user with the highest count
    rospy.loginfo('Highest count is at Robot {}. Sees {} robots'.format(desired_leader + 1, inrange_count[desired_leader]))
    rospy.loginfo('All range counts are: {}'.format(inrange_count))

    rangePub.publish( Int32MultiArray(data=inrange_count))
    leaderPub.publish( desired_leader )
    rospy.loginfo('inrange_count is now published')

###############################################################################
# Range Detector Simulator Functions
###############################################################################
def check_points_in_range(current_rob_pos, other_rob_pos):
    # calculate bottom right corner of the bounding rectangle
    rx, ry, rth = current_rob_pos
    rth = (rth/100) + (pi/2)
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
        rospy.init_node('range_sensor_sim_node')
        rospy.loginfo('%s started' % rospy.get_name())

        # get the necessary global parameters
        grid_dimension = rospy.get_param('/grid_dimension')
        grid_blocks = rospy.get_param('/grid_blocks')
        visible_blocks = rospy.get_param('/visibility_blocks')
        m = grid_dimension * visible_blocks

        # initialize subscribers to positions
        rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, rob1_position_callback)
        rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, rob2_position_callback)
        rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, rob3_position_callback)
        rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, rob4_position_callback)
        rospy.Subscriber('rerun_range_sensors', Byte, rerun_range_sensors_callback)

        # Sleep some time to ensure all subscribtions are working and updated their variables
        print ("Sleeping for %d seconds to ensure subscribers got data" %system_latency)
        time.sleep(system_latency)

        # initialize publisher
        leaderPub = rospy.Publisher('potential_leader', Int32, queue_size=5)
        rangePub = rospy.Publisher('inrange_count', Int32MultiArray, queue_size=10)

        # run the checking algorithm once
        check_in_range_rob1()
        check_in_range_rob2()
        check_in_range_rob3()
        check_in_range_rob4()

        # the main function of the node    
        check_leader()

        rospy.loginfo('Node will go to sleep mode, until a rerun is issued')
        try:
            while not rospy.is_shutdown():
                rospy.spin()

        except KeyboardInterrupt:
            rospy.loginfo('Keyboard Interrupt Exception')
    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
