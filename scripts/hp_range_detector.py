#!/usr/bin/env python
from __future__ import division
import rospy
from math import sin, cos, pi
from std_msgs.msg import Int32, Int32MultiArray, Byte, String
import numpy as np
import transformations
import time

# m is the range sensor maximum distance in cm
grid_dimension = 17.5
grid_blocks = 10
visible_blocks = 3
m = grid_dimension * visible_blocks

# robot position variables
rob1_position = [0,0,0]
rob2_position = [0,0,0]
rob3_position = [0,0,0]
rob4_position = [0,0,0]

# list showing how many robots are in range for robot i
inrange_count = [0,0,0,0]

# shape parameters and robot position variables
current_shape = '-1'
current_shape_length = -1
current_leader_pos = [-1,-1,-1]
possibile_flag = 1

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
# Shape Parameters and Robot Status Callback
###############################################################################
def formation_name_callback(data):
    global current_shape
    current_shape = data.data

def current_shape_length_callback(data):
    global current_shape_length
    current_shape_length = data.data

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
    Function loops over the inrange_count list, checks who sees the most
    robots in the system and assigns them as a leader

    - If 2 robots see the same number, the first one is assigned as a leader
    - After assignment, the function checks that a formation is possible at the current
    leader
    """
    global current_rob_status, current_leader, possibile_flag
    # check which robot is the leader
    desired_leader = 0
    max = 0
    for i in range( len(inrange_count) ):
        if ( inrange_count[i] > max ):
            max = inrange_count[i]
            desired_leader = i

    # check if a formation is possible given the current situation
    # create a list of lists of all positions to fetch the leader positions
    all_positions = [ rob1_position, rob2_position, rob3_position, rob4_position]
    current_leader_pos = all_positions[desired_leader]

    # check that increments in any direction from the leader position will not
    # exceed the test rig boundaries
    x_positive_offset = current_leader_pos[0] + current_shape_length
    x_negative_offset = current_leader_pos[1] - current_shape_length
    y_positive_offset = current_leader_pos[0] + current_shape_length
    y_negative_offset = current_leader_pos[1] - current_shape_length

    # diagonal formation is special because it has side lengths that can be
    # greater than grid size, so we test that first
    if current_shape == 'diagonal':
        sin_45 = sin(pi/4)
        cos_45 = cos(pi/4)

        x_positive_offset = current_leader_pos[0] + ( current_shape_length * cos_45)
        x_negative_offset = current_leader_pos[1] - ( current_shape_length * cos_45)
        y_positive_offset = current_leader_pos[0] + ( current_shape_length * sin_45)
        y_negative_offset = current_leader_pos[1] - ( current_shape_length * sin_45)

        if (( x_positive_offset <= (grid_dimension * grid_blocks) )  and ( y_positive_offset <= (grid_dimension * grid_blocks) )):
            rospy.loginfo('Diagonal formation is POSSIBLE in +ve direction with ROBOT {} as leader'.format(desired_leader +1))
        elif (( x_negative_offset <= (grid_dimension * grid_blocks) ) and ( y_negative_offset <= (grid_dimension * grid_blocks) )):
            rospy.loginfo('Diagonal formation is POSSIBLE in -ve direction with ROBOT {} as leader'.format(desired_leader +1))
        else:
            rospy.loginfo('Diagonal formation is NOT POSSIBLE with ROBOT {} as leader in current position'.format(desired_leader +1))
            # raise a flag for the formation to move the robot in another position
            possibile_flag = 0

    # other shapes are tested here
    else:
        # since most shapes are horizontal or vertical, we test that the positions are within any boundary of the
        # grid. The (-1) for the blocks is because the robot can't get near the reference tag 
        if current_shape_length > grid_dimension * (grid_blocks-1):
            rospy.logwarn('FORMATION IS ABSOLUTELY NOT POSSIBLE WITH CURRENT SHAPE LENGTH')
            rospy.logwarn('NO LEADER WILL BE SELECTED. PLEASE CHANGE PARAMETERS\n')
            rospy.signal_shutdown("No possible formation")

        if current_shape == 'square':
            if ( x_positive_offset <= (grid_dimension * grid_blocks) ) and ( x_negative_offset > grid_dimension ) and ( y_positive_offset <= (grid_dimension * grid_blocks) ) and ( y_negative_offset > grid_dimension ):
                rospy.loginfo('Square formation is POSSIBLE with ROBOT {} as leader'.format(desired_leader +1))
            else:
                rospy.loginfo('Square formation is NOT POSSIBLE with ROBOT {} as leader in current position'.format(desired_leader +1))
                # raise a flag for the formation to move the robot in another position
                possibile_flag = 0

        if current_shape == 'line':
            if ( x_positive_offset <= (grid_dimension * grid_blocks) ) and ( x_negative_offset > grid_dimension ):
                rospy.loginfo('Line formation is POSSIBLE with ROBOT {} as leader'.format(desired_leader +1))

            else:
                rospy.loginfo('Line formation is NOT POSSIBLE with ROBOT {} as leader in current position'.format(desired_leader +1))
                # raise a flag for the formation to move the robot in another position
                possibile_flag = 0

        if current_shape == 'column':
            if ( y_positive_offset <= (grid_dimension * grid_blocks) ) and ( y_negative_offset > grid_dimension ):
                rospy.loginfo('Column formation is POSSIBLE with ROBOT {} as leader'.format(desired_leader +1))
            else:
                rospy.loginfo('Column formation is NOT POSSIBLE with ROBOT {} as leader in current position'.format(desired_leader +1))
                # raise a flag for the formation to move the robot in another position
                possibile_flag = 0

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
        rospy.init_node('hp_range_detector')
        rospy.loginfo('%s started' % rospy.get_name())

        # initialize subscribers to positions
        rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, rob1_position_callback)
        rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, rob2_position_callback)
        rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, rob3_position_callback)
        rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, rob4_position_callback)
        rospy.Subscriber('rerun_range_sensors', Byte, rerun_range_sensors_callback)

        # subscribers to shape parameters and robot status
        rospy.Subscriber('formation_shape_name', String, formation_name_callback)
        rospy.Subscriber('shape_length', Int32, current_shape_length_callback)

        # Sleep some time to ensure all subscribtions are working and updated their variables
        time.sleep(5)

        # initialize publisher
        rangePub = rospy.Publisher('inrange_count', Int32MultiArray, queue_size=10)
        possibleFormation = rospy.Publisher('formation_possible_flag', Byte, queue_size=10)

        # run the checking algorithm once
        check_in_range_rob1()
        check_in_range_rob2()
        check_in_range_rob3()
        check_in_range_rob4()
        set_and_check_leader()

        # stay in a loop publishing until another run is issues via the subscriber
        # function will sleep 1 minute to save CPU power
        try:
            while not rospy.is_shutdown():
                rangePub.publish( Int32MultiArray(data=inrange_count) )
                possibleFormation.publish( possibile_flag )
                #Sleeps for 1 Minute - Used to reduce CPU usage
                rospy.loginfo('Sleeping for a minute to save power')
                rospy.sleep(60)

        except KeyboardInterrupt:
            rospy.loginfo('Keyboard Interrupt Exception')

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
