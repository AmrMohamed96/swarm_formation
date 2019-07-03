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
visible_blocks = 4
m = grid_dimension * visible_blocks

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
possibile_flag = 0

checking_loop_count = 0
first_leader_assigned = 0
possible_leader = 0
recursion_flag = 0

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
def check_formation_at_leader(leader_id):
    global checking_loop_count, possibile_flag, possible_leader, recursion_flag
    current_eval_leader = leader_id
    checking_loop_count +=1
    possibile_flag = 0

    # base case for recursions:
    if checking_loop_count < 5:
        rospy.loginfo('Checking formation possibility at leader: %d' %(leader_id+1))
        all_positions = [ rob1_position, rob2_position, rob3_position, rob4_position]
        current_leader_pos = all_positions[leader_id]

        if current_shape_length > 17.5:
            # checking if square shape is possible in which quad
            if current_shape == 'square':
                # 1st quad
                x_max = current_leader_pos[0] + current_shape_length
                y_max = current_leader_pos[1] + current_shape_length

                if (x_max < grid_dimension * grid_blocks) and (y_max < grid_dimension * grid_blocks):
                    possibile_flag = 1
            
                x_min = current_leader_pos[0] - current_shape_length
                y_min = current_leader_pos[1] - current_shape_length

                if (x_min > 0) and (y_min > 0):
                    possibile_flag +=2
        else:
            rospy.logerr('Shape length is too small')
            return 0
        
        if possibile_flag == 1:
            rospy.loginfo('Formation possible in 1st quad only')
            possible_leader = current_eval_leader 
            recursion_flag = 1
            return 1

        elif possibile_flag == 2:
            rospy.loginfo('Formation possible in 4th quad only')
            possible_leader = current_eval_leader
            recursion_flag = 1 
            return 1

        elif possibile_flag == 3:
            rospy.loginfo('Formation possible in 1st and 4th quad')
            possible_leader = current_eval_leader
            recursion_flag = 1
            return 1

        elif possibile_flag == 0:
            rospy.logwarn('Formation is not possible with Robot [%d] as leader. Checkanother one'% (leader_id+1))
            check_formation_at_leader( ( first_leader_assigned + checking_loop_count )%4 )

    else:
        rospy.logerr('FORMATION NOT AT ALL POSSIBLE')
        return 0

def set_and_check_leader():
    """
    Function loops over the inrange_count list, checks who sees the most
    robots in the system and assigns them as a leader

    - If 2 robots see the same number, the first one is assigned as a leader
    - After assignment, the function checks that a formation is possible at the current
    leader
    """
    global current_rob_status, current_leader, possibile_flag, first_leader_assigned, possible_leader
    # check which robot is the leader
    desired_leader = 0
    max = 0
    for i in range( len(inrange_count) ):
        if ( inrange_count[i] > max ):
            max = inrange_count[i]
            desired_leader = i

    # set the leader
    setLead = rospy.Publisher('set_new_leader', Byte, queue_size=1)
    time.sleep(system_latency)
    rospy.loginfo('Leader will be Robot {} as it sees {} robots'.format(desired_leader + 1, inrange_count[desired_leader]))
    
    first_leader_assigned = desired_leader

    check_formation_at_leader(desired_leader)

    print "Recursion flag: ", recursion_flag
    if recursion_flag:
        # publish the leader ID
        print "Published id = ", possible_leader
        setLead.publish(possible_leader+1)
        possibleFormation.publish(possibile_flag)

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
        while current_shape == '':
            rospy.Subscriber('formation_shape_name', String, formation_name_callback)
            rospy.sleep(5)
            
        set_and_check_leader()

        # stay in a loop publishing until another run is issues via the subscriber
        # function will sleep 1 minute to save CPU power

        Rate = rospy.Rate(0.5)

        try:
            while not rospy.is_shutdown():
                rangePub.publish( Int32MultiArray(data=inrange_count) )
                possibleFormation.publish( possibile_flag )
                rangePub.publish( Int32MultiArray(data=inrange_count))
                Rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo('Keyboard Interrupt Exception')

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
