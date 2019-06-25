#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32MultiArray, Int32, Byte
import numpy as np

"""
    GLOBAL KNOWLEDGE NODE FOR FORMATION ALGORITHM
    DATA:
    - the node contains data that need to be shared accross all
    robots inside the system

    FUNCTIONS:
    - update robot status to leader
    - update robot status to follower1, follower2
    - update next goals for each robot
"""
###############################################################################
# Global variables that will continously be published
###############################################################################
# a list of list to show status of alignment of each robot
align_axis = [ [0,0],[0,0],[0,0],[0,0] ]

# a list of lists that shows the coordinates of robots forming corners
shape_corner_robots = [ [0,0],[0,0],[0,0],[0,0] ]

# pattern estimation parameters
shape_length = 0
shape_sides = 0
x_sides = [0,0,0,0]
y_sides = [0,0,0,0]

# logging parameters that we introduced to the global node
# 1: Leader, 2: Follower1, 3: Follower2
robot_status = [0,0,0,0]
prev_robot_status = [0,0,0,0]
# Rob Num: Leader Count
leader_history = [0,0,0,0]
# next goals for all robots, index indicates robot number
next_goals = [ [0,0], [0,0], [0,0], [0,0] ]

###############################################################################
# Callbacks that take align axis from each robot and update the global align axis
###############################################################################
def edit_align_rob1_callback(data):
    global align_axis
    align_axis[0][0] = data.data[0]
    align_axis[0][1] = data.data[1]

def edit_align_rob2_callback(data):
    global align_axis
    align_axis[1][0] = data.data[0]
    align_axis[1][1] = data.data[1]

def edit_align_rob3_callback(data):
    global align_axis
    align_axis[2][0] = data.data[0]
    align_axis[2][1] = data.data[1]

def edit_align_rob4_callback(data):
    global align_axis
    align_axis[3][0] = data.data[0]
    align_axis[3][1] = data.data[1]

###############################################################################
# Callbacks that take coordinates of corner robots and update the global list
###############################################################################
def edit_corners_rob1_callback(data):
    global shape_corner_robots
    shape_corner_robots[0][0] = data.data[0]
    shape_corner_robots[0][1] = data.data[1]

def edit_corners_rob2_callback(data):
    global shape_corner_robots
    shape_corner_robots[1][0] = data.data[0]
    shape_corner_robots[1][1] = data.data[1]

def edit_corners_rob3_callback(data):
    global shape_corner_robots
    shape_corner_robots[2][0] = data.data[0]
    shape_corner_robots[2][1] = data.data[1]

def edit_corners_rob4_callback(data):
    global shape_corner_robots
    shape_corner_robots[3][0] = data.data[0]
    shape_corner_robots[3][1] = data.data[1]

###############################################################################
# Function to set the leader status
###############################################################################
def set_new_leader_callback(data):
    global robot_status, prev_robot_status, leader_history
    robot_status = [0,0,0,0]
    robot_status[data.data - 1] = 1
    if prev_robot_status != robot_status:
        leader_history[data.data -1] += 1
        rospy.loginfo("New Leader: Robot " + str(data.data) + " is now leader")
    prev_robot_status = robot_status

###############################################################################
# Function to reset the leader of the system
###############################################################################
def reset_leader_stats_callback(data):
    global robot_status
    if (data.data == 1):
        robot_status = [9,9,9,9]
        rospy.loginfo("Leaders has been reset. Execution of follower 4 routine ready")

###############################################################################
# Function to update followers status
###############################################################################
def assign_followers_callback(data):
    global robot_status
    robot_status = data.data

###############################################################################
# Function to update next goal of robot i
###############################################################################
def update_next_goals_callback(data):
    global next_goals
    next_goals = list()
    next_goals = next_goals.append( [data.data[0], data.data[1]] )
    next_goals = next_goals.append( [data.data[2], data.data[3]] )
    next_goals = next_goals.append( [data.data[4], data.data[5]] )
    next_goals = next_goals.append( [data.data[6], data.data[7]] )

###############################################################################
# Set Pattern Parameters Callback
###############################################################################
def set_pattern_param_callback(data):
    global shape_length, shape_sides, x_sides, y_sides
    shape_length = data.data[0]
    shape_sides = data.data[1]

    x_sides = list()
    x_sides = x_sides.append( [ data.data[2], data.data[3] ])
    x_sides = x_sides.append( [ data.data[4], data.data[5] ])

    y_sides = list()
    y_sides = y_sides.append( [ data.data[6], data.data[7] ])
    y_sides = y_sides.append( [ data.data[8], data.data[9] ])

###############################################################################
# Subscribers of the global node
###############################################################################
def global_node_listener():
    # subscribers that update align axis list of each robot independently
    rospy.Subscriber('align_axis_rob1', Int32MultiArray, edit_align_rob1_callback)
    rospy.Subscriber('align_axis_rob2', Int32MultiArray, edit_align_rob2_callback)
    rospy.Subscriber('align_axis_rob3', Int32MultiArray, edit_align_rob3_callback)
    rospy.Subscriber('align_axis_rob4', Int32MultiArray, edit_align_rob4_callback)

    # subscribers that update shape corner robots
    rospy.Subscriber('shape_corner_rob1', Int32MultiArray, edit_corners_rob1_callback)
    rospy.Subscriber('shape_corner_rob2', Int32MultiArray, edit_corners_rob2_callback)
    rospy.Subscriber('shape_corner_rob3', Int32MultiArray, edit_corners_rob3_callback)
    rospy.Subscriber('shape_corner_rob4', Int32MultiArray, edit_corners_rob4_callback)

    # subscriber that takes the pattern parameters (Shape_Length, Shape_Sides, X_Sides, Y_Sides)
    rospy.Subscriber('pattern_parameters', Int32MultiArray, set_pattern_param_callback)

    # subscriber that updates followers
    rospy.Subscriber('assign_follower_robots',Int32MultiArray, assign_followers_callback)

    # subscriber that updates next goals
    rospy.Subscriber('update_next_goals', Int32MultiArray, update_next_goals_callback)

    # subscriber to set a leader
    rospy.Subscriber('set_new_leader', Byte, set_new_leader_callback)

    # subscriber to reset the leader
    rospy.Subscriber('reset_leader_stats', Byte, reset_leader_stats_callback)

###############################################################################
# Main code of the global node
###############################################################################
def global_talker():
    global leader_history
    # Defining Publishers
    alignPub = rospy.Publisher('align_axis', Int32MultiArray, queue_size=10)
    shapeCorners = rospy.Publisher('shape_corner_robots', Int32MultiArray, queue_size=10)

    shapeLength = rospy.Publisher('shape_length', Int32, queue_size=10)
    shapeSides = rospy.Publisher('shape_sides', Int32, queue_size=10)
    xSides = rospy.Publisher('x_sides', Int32MultiArray, queue_size=10)
    ySides = rospy.Publisher('y_sides', Int32MultiArray, queue_size=10)

    robotStats = rospy.Publisher('robot_status', Int32MultiArray, queue_size=10)
    leaderHistory = rospy.Publisher('leader_history', Int32MultiArray, queue_size=10)
    nextGoals = rospy.Publisher('next_goals', Int32MultiArray, queue_size=10)

    # Starting the subscribers
    global_node_listener()

    try:
        while not rospy.is_shutdown():
            # broadcasting all nodes data
            alignPub.publish( Int32MultiArray(data = np.reshape(align_axis, (8)) ))
            shapeCorners.publish( Int32MultiArray(data = np.reshape(shape_corner_robots, (8))) )
            shapeLength.publish(shape_length)
            shapeSides.publish(shape_sides)
            xSides.publish( Int32MultiArray(data=x_sides) )
            ySides.publish( Int32MultiArray(data=y_sides) )

            robotStats.publish( Int32MultiArray(data=robot_status) )
            leaderHistory.publish( Int32MultiArray(data=leader_history) )
            nextGoals.publish( Int32MultiArray( data= np.reshape(next_goals, (8)) ) )

    except KeyboardInterrupt:
        rospy.loginfo("Node was forced to close")

if __name__ == '__main__':
    try:
        # initializing the global node
        rospy.init_node('formation_global_knowledge')
        rospy.loginfo("%s started" % rospy.get_name())
        global_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
