#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Int32MultiArray, Byte, String
import numpy as np

# other robots positions
obst1 = [0,0,0]
obst2 = [0,0,0]

# current robot positions
rob1 = [0,0,0]
rob2 = [0,0,0]
rob3 = [0,0,0]
rob4 = [0,0,0]

# grid dimension
grid_dim = 0
grid_blocks = 0

# formation param
shape_length = 0
shape_name = ''

# potential leader
potential_leader = 0

quadPub = rospy.Publisher('formation_quad', Int32, latch=True, queue_size=10)
leadPub = rospy.Publisher('set_new_leader', Byte, queue_size=10)

def quad_checker_square(leader_id):
    global selected_quad

    quad1_valid = 0
    quad2_valid = 0
    quad3_valid = 0
    quad4_valid = 0

    while shape_name == '' :
        pass

    # add all positions of robots in list of list
    all_pos = [ rob1, rob2, rob3, rob4]
    current_point = all_pos[leader_id]

    corner_points = np.zeros( (3,3))

    # check 1st quad
    corner_points[0] = current_point
    corner_points[0][1] += shape_length

    corner_points[1] = current_point
    corner_points[1][0] += shape_length

    corner_points[2] = current_point
    corner_points[2][0] += shape_length
    corner_points[2][1] += shape_length

    for i in range( 3 ):
        if ( corner_points[i][0] > (grid_dim * grid_blocks) ) or ( corner_points[i][1] > (grid_dim * grid_blocks) ) or ( corner_points[i][0] < 0)  or ( corner_points[i][0] < 0 ):
            if ( (corner_points[i][0] == obst1[0] ) and (corner_points[i][1] == obst1[0] ) ) or ( (corner_points[i][0] == obst2[0] ) and (corner_points[i][1] == obst2[0] ) ):
                continue
        else:
            quad1_valid +=1
    
    if quad1_valid == 3:
        return 1

    # check 2nd quad
    corner_points[0] = current_point
    corner_points[0][1] += shape_length

    corner_points[1] = current_point
    corner_points[1][0] -= shape_length

    corner_points[2] = current_point
    corner_points[2][0] -= shape_length
    corner_points[2][1] += shape_length

    for i in range( len(corner_points) ):
        if ( corner_points[i][0] > (grid_dim * grid_blocks) ) or ( corner_points[i][1] > (grid_dim * grid_blocks) ) or ( corner_points[i][0] < 0)  or ( corner_points[i][0] < 0 ):
            if ( (corner_points[i][0] == obst1[0] ) and (corner_points[i][1] == obst1[0] ) ) or ( (corner_points[i][0] == obst2[0] ) and (corner_points[i][1] == obst2[0] ) ):
                continue
        else:
            quad2_valid += 1
    
    if quad2_valid == 3:
        return 2

    # check 3rd quad
    corner_points[0] = current_point
    corner_points[0][1] -= shape_length

    corner_points[1] = current_point
    corner_points[1][0] -= shape_length

    corner_points[2] = current_point
    corner_points[2][0] -= shape_length
    corner_points[2][1] -= shape_length

    for i in range( 3 ):
        if ( corner_points[i][0] > (grid_dim * grid_blocks) ) or ( corner_points[i][1] > (grid_dim * grid_blocks) ) or ( corner_points[i][0] < 0)  or ( corner_points[i][0] < 0 ):
            if ( (corner_points[i][0] == obst1[0] ) and (corner_points[i][1] == obst1[0] ) ) or ( (corner_points[i][0] == obst2[0] ) and (corner_points[i][1] == obst2[0] ) ):
                continue
        else:
            quad3_valid += 1
    
    if quad3_valid == 3:
        return 3

    # check 4th quad
    corner_points[0] = current_point
    corner_points[0][1] -= shape_length

    corner_points[1] = current_point
    corner_points[1][0] += shape_length

    corner_points[2] = current_point
    corner_points[2][0] += shape_length
    corner_points[2][1] -= shape_length

    for i in range( 3 ):
        if ( corner_points[i][0] > (grid_dim * grid_blocks) ) or ( corner_points[i][1] > (grid_dim * grid_blocks) ) or ( corner_points[i][0] < 0)  or ( corner_points[i][0] < 0 ):
            if ( (corner_points[i][0] == obst1[0] ) and (corner_points[i][1] == obst1[0] ) ) or ( (corner_points[i][0] == obst2[0] ) and (corner_points[i][1] == obst2[0] ) ):
                continue
        else:
            quad3_valid += 1

    if quad4_valid == 3:
        return 4

    return 0

def rob1_callback(data):
    global rob1
    rob1 = data.data

def rob2_callback(data):
    global rob2
    rob2 = data.data

def rob3_callback(data):
    global rob3
    rob3 = data.data

def rob4_callback(data):
    global rob4
    rob4 = data.data

def obst1_callback(data):
    global obst1
    obst1 = data.data

def obst2_callback(data):
    global obst2
    obst2 = data.data

def shape_name_callback(data):
    global shape_name
    shape_name = data.data 

def shape_length_callback(data):
    global shape_length
    shape_length = data.data 

def potential_leader_callback(data):
    global potential_leader, selected_quad
    potential_leader = data.data

    selected_quad, selected_leader = check_routine(potential_leader)
    quadPub.publish(selected_quad)
    leadPub.publish(selected_leader+1)
    rospy.loginfo('Square formation is possible with R {} in {} Quad'.format(selected_leader+1, selected_quad))

def check_routine(leader_id):
    select = leader_id
    for i in range (3):
        valid_quad = quad_checker_square(select)
        if valid_quad:
            return valid_quad, select
        select = ((leader_id +1)) % 4
    return 0
        

def subscribers():
    # current robot position
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, rob1_callback)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, rob2_callback)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, rob3_callback)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, rob4_callback)

    # other robots and obstacles positions
    rospy.Subscriber('obst1', Int32MultiArray, obst1_callback)
    rospy.Subscriber('obst2', Int32MultiArray, obst2_callback)

    # shape parameters
    rospy.Subscriber('formation_shape_name', String, shape_name_callback)
    rospy.Subscriber('shape_length', Int32, shape_length_callback)

    # current leader param
    rospy.Subscriber('potential_leader', Int32, potential_leader_callback)

if __name__ == '__main__':
    # initalize the ros node
    rospy.init_node('leader_selector')
    rospy.loginfo('%s started' % rospy.get_name())

    # grid dimension param
    grid_dim = rospy.get_param('/grid_dimension')
    grid_blocks = rospy.get_param('/grid_blocks') - 1

    subscribers()

    # unless a shape is not defined do nothing

    # spin parameter
    while not rospy.is_shutdown():
        rospy.spin()