#!/usr/bin/env python

import argparse
import sys
import rospy
from std_msgs.msg import Int32MultiArray, Int32, String, Float32, Byte
import numpy
import math
import os
import time
###############################################################################
#initialize all variables:
###############################################################################
#desired orientation difference:
Ad2 =Ad3 = 0
y=0
#desired distance:
Dd=0 #desired distance of the shape
c2c_distance_px=1 # it is published from the user input and if not it by default =1
grid_dim=17.5 #cm
grid_dig_dim=24.75 #cm
swarm_robots_num=4 # this year we have 4 robots
#orientation in radians:
A2=A3=0
#formation shape:
shapes = ''
#leader new position after check in cm:
R1_gx_cm = R1_gy_cm =0
#leader final position
rob1_goal_x =rob1_goal_y =0
#leader current position:
x1=y1=a1=0
#robot 2 current position:
x2=y2=a2=0
#robot 3 current position:
x3=y3=a3=0
#robot 4 current position:
x4=y4=a4=0
#incrementation values:
m=n=0
all_status=[0,0,0,0]
poses=[0,0,0,0,0,0,0,0]

# calculations flag
leader_calc_flag = False
###############################################################################
#RobotClass:
###############################################################################
class RobotClass:
    ID= -1
    status=-1
    def __init__(self,status=0):
        self.ID = rospy.get_param('~robot_id', 1)
        self.status=status

    def set_ID(self, ID):
        self.ID = ID
    def set_status(self, status):
        self.status =status
    def get_ID(self):
        return self.ID
    def get_status(self):
        return self.status

###############################################################################
#define publishers:
###############################################################################
pub_rob1_goal_px  = rospy.Publisher('robot1_goal_px', Int32MultiArray, queue_size=10)

pub_rob2_goal_px = rospy.Publisher('robot2_goal_px', Int32MultiArray, queue_size=10)
pub_rob3_goal_px = rospy.Publisher('robot3_goal_px', Int32MultiArray, queue_size=10)
pub_rob4_goal_px = rospy.Publisher('robot4_goal_px', Int32MultiArray, queue_size=10)

pub_rob2_goal_cm = rospy.Publisher('robot2_goal_cm', Int32MultiArray, queue_size=10)
pub_rob3_goal_cm = rospy.Publisher('robot3_goal_cm', Int32MultiArray, queue_size=10)
pub_rob4_goal_cm = rospy.Publisher('robot4_goal_cm', Int32MultiArray, queue_size=10)

pub_assign_follower_robots = rospy.Publisher('assign_follower_robots', Int32MultiArray, queue_size=10)
pub_update_next_goals = rospy.Publisher('update_next_goals',Int32MultiArray,queue_size=10)


pub_m_n_values = rospy.Publisher('m_n_values', Int32MultiArray, queue_size=10)
###############################################################################
#callback functions:
###############################################################################
def callback_status (data):
    global R, all_status
    R.set_status(data.data[R.ID-1])
    all_status=list()
    all_status = [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_align_axis(data):
    global align_axis
    align_axis = numpy.reshape(data.data, (4,2))

def callback_shape_corner_robots(data):
    global shape_corner_robots
    shape_corner_robots = numpy.reshape(data.data, (4,2))

def callback_next_goal(data):
    global next_goal
    next_goal = numpy.reshape(data.data, (4,2))

def callback_x_sides(data):
    global x_sides
    x_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_y_sides(data):
    global y_sides
    y_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_shape_sides(data):
    global shape_sides
    shape_sides = data.data

def callback_shape_length(data):
    global shape_length
    shape_length = data.data

def callback_g2g_flag_rob (data):
    global g2g_flag
    g2g_flag=data.data

def callback_formation_shape(data): # formation shape:
    global shapes
    shapes = data.data

def callback_c2c_distance_px(data): #center to center distance in pixels
    global c2c_distance_px
    c2c_distance_px = data.data

def callback_rob1_current_pos(data): #current position of robot1
    global x1, y1, a1
    #rospy.loginfo('robot1 co. = %s', data.data)
    x1 = data.data[0]
    y1 = data.data[1]
    a1 = data.data[2]

def callback_rob2_current_pos(data): #current position of robot 2
    global x2, y2, a2
    #rospy.loginfo('robot2 co. = %s', data.data)
    x2 = data.data[0]
    y2 = data.data[1]
    a2 = data.data[2]

def callback_rob3_current_pos(data): #current position of robot 3
    global x3, y3, a3
    #rospy.loginfo('robot3 co. = %s', data.data)
    x3 = data.data[0]
    y3 = data.data[1]
    a3 = data.data[2]

def callback_rob4_current_pos(data): #current position of robot 4
    global x4, y4, a3
    #rospy.loginfo('robot4 co. = %s', data.data)
    x4 = data.data[0]
    y4 = data.data[1]
    a4 = data.data[2]
    shape()
    final()

def callback_robots_current_poses(data):
    ''' this call back fn takes all current poses from camera direct in one array
    '''
    global poses
    poses = numpy.reshape(data.data, (4,2))

def callback_leader_final_goal(data): # new position of the leader after check
    global R1_gx_cm, R1_gy_cm
    #rospy.loginfo('robot3 co. = %s', data.data)
    R1_gx_cm = data.data[0]
    R1_gy_cm = data.data[1]

###############################################################################
#who_am_I function:
###############################################################################
def who_am_I():
    ''' this function is run ater subscription to all the robot data needed and
        set flag to 1 for successfully robot identification
    '''
    global R
    R=RobotClass()
    pub_align_rob = rospy.Publisher('align_rob'+str(R.ID), Int32MultiArray,queue_size=10)
    pub_corners_rob =rospy.Publisher('corners_rob'+str(R.ID), Int32MultiArray,queue_size=10)
    rospy.Subscriber('robot_status',Int32MultiArray,callback_status)
    time.sleep(1)
    print "I am robot", R.ID , "with status:" , R.status

###############################################################################
#define subscribers:
###############################################################################
def listener():
    rospy.Subscriber('formation_shape_name', String, callback_formation_shape)
    #rospy.Subscriber('c2c_distance_px',Int32, callback_c2c_distance_px)

    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback_rob1_current_pos)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, callback_rob2_current_pos)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, callback_rob3_current_pos)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, callback_rob4_current_pos)
    rospy.Subscriber('robots_current_poses', Int32MultiArray, callback_robots_current_poses)

    rospy.Subscriber('rob1_final_cm', Int32MultiArray, callback_leader_final_goal)
    rospy.Subscriber('g2g_flag_rob'+str(R.ID), Int32, callback_g2g_flag_rob)

def listener_from_global():
    rospy.Subscriber('align_axis',Int32MultiArray,callback_align_axis)
    rospy.Subscriber('shape_corner_robots',Int32MultiArray,callback_shape_corner_robots)
    rospy.Subscriber('next_goals',Int32MultiArray,callback_next_goal)
    rospy.Subscriber('x_sides',Int32MultiArray,callback_x_sides)
    rospy.Subscriber('y_sides',Int32MultiArray,callback_y_sides)
    rospy.Subscriber('shape_sides',Int32,callback_shape_sides)
    rospy.Subscriber('shape_length',Int32,callback_shape_length)

###############################################################################
#justify_distance function:
###############################################################################
def justify_distance(next,base,length):
    ''' (next) point is used as the follower point,
        (base) is used as leader point,
        (length) is used as shape length
        this function justify the goal position of the robot to the shape length
        by checking if the desired position above/below shape length
        then the goal is updated accordingly to a point that
        meet the shape length requirement.
    '''
    #1. initialize justified Point to 0,0:
    justified_point=[0,0]
    #2. check that next and bas sharing the same x:
    if next[0]==base[0]:
        justified_point[0]= base[0]
        if base[1]<next[1]: #next is above the base
            justified_point[1]=base[1]+length
        else: #next is below
            justified_point[1]=base[1]-length
    #3. check that next and bas sharing the same y:
    elif next[1]==base[1]:
        justified_point[1]= base[1]
        if base[0]<next[0]: #next is above the base
            justified_point[0]=base[0]+length
        else:
            justified_point[0]=base[0]-length
    else:
        print "no justify yet(error)"
    return justified_point

###############################################################################
#align function:
###############################################################################
def align(follower,follower_num,leader,leader_num,direction):
    ''' this function determine the goal position for follower based on alignment direction.
        The robot align in 90 degree in x or y direction. The
        function takes direction, follower and leader position anddetermine the next goal point according to direction
        specified. The resulted goal position is then modified
        according to shape length by calling a justify_distance
        function. align_axis global flags are updated after goal
        position is chosen for both follower and leader
    '''
    global next_goal, align_axis
    #align in x or y as demand:
    if (direction=='x'):
        #1. align the next_goal x index with leader x index:
        next_goal[follower_num][1]=leader[1]
        #2. leave the y index of the next_goal as the follower:
        next_goal[follower_num][0]=follower[0]
        #3. set the align axis:
        align_axis[leader_num][0]=1
        align_axis[follower_num][0]=1
    elif (direction=='y'):
        #1. align the next_goal y index with leader y index:
        next_goal[follower_num][0]=leader[0]
        #2. leave the x index of the next_goal as the follower:
        next_goal[follower_num][1]=follower[1]
        #3. set the align axis:
        align_axis[leader_num][1]=1
        align_axis[follower_num][1]=1
    #update next_goal of the follower:
    next_goal[follower_num]=justify_distance(next_goal[follower_num],leader,shape_length)

###############################################################################
#shape function:
###############################################################################
def shape():
    ''' this function checks what formation is sent and set
        the orientations and distance between the robots desired.
        also set the incrementations the leader should move if obstacles found
    '''
    global Ad2, Ad3, Ad4, Dd, shapes, m, n #line,column,diagonal
    if shapes == "line" :
        Ad2=0
        Ad3=180
        #Ad4=0
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=3
        n=0
    elif shapes == "column" :
        Ad2=90
        Ad3=270
        #Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=0
        n=3
    elif shapes == "diagonal" :
        Ad2=45
        Ad3=-135
        #Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=3
        n=0
    elif shapes == "triangle":
        Ad2=-45
        Ad3=-135
        #Ad4=270
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=2
        n=0
    elif shapes == "L-shape":
        Ad2=0
        Ad3=90
        #Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    elif shapes == "square":
        Ad2=0
        Ad3=90
        #Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    else:
        print "shape is not defined"

    #publish the incrementation values of leader
    m_n_values = numpy.array([ m ,n],Int32MultiArray)
    m_n_values_=Int32MultiArray(data=m_n_values)
    pub_m_n_values.publish(m_n_values_)

###############################################################################
#calculation function:
###############################################################################
def calculations():
    ''' This function is to calculate the the follower1 and follower2 goal
        position based on leader position and the shape given
    '''
    global A2, A3 ,f13_CM ,f12_CM ,rob3_goal ,rob2_goal ,rob2_goal_px ,rob3_goal_px
    #change the angels into radians
    A2 = Ad2*(math.pi/180)
    A3 = Ad3*(math.pi/180)

    #calculate positions of follower 1 and 2 which is robot 2 and 3:
    rob2_goal_x = int(round(rob1_goal_x + (Dd*math.cos(A2))))
    rob3_goal_x = int(round(rob1_goal_x + (Dd*math.cos(A3))))

    rob2_goal_y = int(round(rob1_goal_y - (Dd*math.sin(A2))))
    rob3_goal_y = int(round(rob1_goal_y - (Dd*math.sin(A3))))

    #print rob2_goal_x ,rob2_goal_y

    #calculate the distances between the current position of the robot and both formations positions
    dA1 = math.sqrt(pow((x2-rob2_goal_x),2)+pow((y2-rob2_goal_y),2))
    dA2 = math.sqrt(pow((x2-rob3_goal_x),2)+pow((y2-rob3_goal_y),2))
    dB1 = math.sqrt(pow((x3-rob2_goal_x),2)+pow((y3-rob2_goal_y),2))
    dB2 = math.sqrt(pow((x3-rob3_goal_x),2)+pow((y3-rob3_goal_y),2))
    #set values of the distances in an array
    DA = [dA1 , dA2]
    DB = [dB1 , dB2]

    #sort the distances to find the longest and shortest distance for each robot
    list.sort(DA)
    list.sort(DB)

    #change the final positions into pixels
    rob2_goal_x_px = math.ceil(((rob2_goal_x * 2.43308) /42.9))-1
    rob2_goal_y_px = math.ceil(((rob2_goal_y * 2.43308) /40.4))-1

    rob3_goal_x_px = math.ceil(((rob3_goal_x * 2.43308) /42.9))-1
    rob3_goal_y_px = math.ceil(((rob3_goal_y * 2.43308) /40.4))-1

    #check for the longest distance any robot would move
    if DA[1] < DB[1]:
        #send the robot the closest position to it
        if (dB2 > dB1):
            rob3_goal = numpy.array([ rob2_goal_x , rob2_goal_y , a1 ],Int32MultiArray)
            rob2_goal = numpy.array([ rob3_goal_x , rob3_goal_y , a1 ],Int32MultiArray)
            rob3_goal_px = numpy.array([ rob2_goal_x_px , rob2_goal_y_px , a1 ],Int32MultiArray)
            rob2_goal_px = numpy.array([ rob3_goal_x_px , rob3_goal_y_px , a1 ],Int32MultiArray)
        else:
            rob2_goal = numpy.array([ rob2_goal_x , rob2_goal_y , a1 ],Int32MultiArray)
            rob3_goal = numpy.array([ rob3_goal_x , rob3_goal_y , a1 ],Int32MultiArray)
            rob2_goal_px = numpy.array([ rob2_goal_x_px , rob2_goal_y_px , a1 ],Int32MultiArray)
            rob3_goal_px = numpy.array([ rob3_goal_x_px , rob3_goal_y_px , a1 ],Int32MultiArray)

    else:
        if (dA2 > dA1):
            rob2_goal = numpy.array([ rob2_goal_x , rob2_goal_y , a1 ],Int32MultiArray)
            rob3_goal = numpy.array([ rob3_goal_x , rob3_goal_y , a1 ],Int32MultiArray)
            rob2_goal_px = numpy.array([ rob2_goal_x_px , rob2_goal_y_px , a1 ],Int32MultiArray)
            rob3_goal_px = numpy.array([ rob3_goal_x_px , rob3_goal_y_px , a1 ],Int32MultiArray)
        else:
            rob3_goal = numpy.array([ rob2_goal_x , rob2_goal_y , a1 ],Int32MultiArray)
            rob2_goal = numpy.array([ rob3_goal_x , rob3_goal_y , a1 ],Int32MultiArray)
            rob3_goal_px = numpy.array([ rob2_goal_x_px , rob2_goal_y_px , a1 ],Int32MultiArray)
            rob2_goal_px = numpy.array([ rob3_goal_x_px , rob3_goal_y_px , a1 ],Int32MultiArray)

    #publish the formation positions:
    f13_=Int32MultiArray(data=rob3_goal_px)
    f12_=Int32MultiArray(data=rob2_goal_px)
    print 'robot2 grid goal' ,rob2_goal_px
    print 'robot3 grid goal' ,rob3_goal_px
    pub_rob2_goal_px.publish(f12_)
    pub_rob3_goal_px.publish(f13_)

    f13_CM=Int32MultiArray(data=rob3_goal)
    f12_CM=Int32MultiArray(data=rob2_goal)
    print 'robot2 goal' ,rob2_goal
    print 'robot3 goal' ,rob3_goal
    pub_rob2_goal_cm.publish(f12_CM)
    pub_rob3_goal_cm.publish(f13_CM)

###############################################################################
#final function:
###############################################################################
def leader_flag_callback(data):
    global leader_goal_flag
    leader_goal_flag = data.data

def follower1_flag_callback(data):
    global follower1_goal_flag
    follower1_goal_flag = data.data

def final():
    ''' this function is run after the whole robots position is subscribed and
        are ready to use in this code.
        the main purpose of this function is to check shape is entered and
        check leader goal is modified or not and calculate the follower1 and 2
        new goals and publish them.
    '''
    if ( shapes != '' ):
        if (R.status==1): #leader
            global x1 ,y1,rob1_goal_x ,rob1_goal_y,rob1_goal_x_px ,rob1_goal_y_px, leader_calc_flag
            leaderGoalFlag = rospy.Publisher('leader_reached_flag', Byte, queue_size=10)
            if not leader_calc_flag:
                #if leader has a new position set the position to be added
                #into the calculations as the new position
                if ((R1_gx_cm > 0) and (R1_gy_cm > 0)):
                    x1= R1_gx_cm
                    y1= R1_gy_cm
                    rob1_goal_x = x1
                    rob1_goal_y = y1
                    rob1_goal_x_px = x1
                    rob1_goal_y_px = y1
                    shape_corner_robots[0]=[rob1_goal_x_px,rob1_goal_y_px]
                    nearest_two_neighbors=find_nearest_two_neighbors(R.ID-1)
                    followers_routine_step1(R.ID-1,nearest_two_neighbors[0],nearest_two_neighbors[1])

                    next_goal_list[(R.ID-1)]= rob1_goal_x
                    next_goal_list[(R.ID)]= rob1_goal_y
                    next_goal_list=list()
                    next_goal_list=numpy.reshape(next_goal,(8))
                    pub_update_next_goals.publish(Int32MultiArray(data= next_goal_list ))

                    move(R.ID)
                    leader_calc_flag = True
                leaderGoalFlag.publish(1)

            else:
                if not leader_calc_flag:
                    #if leader is not supposed to move set the positions to be added
                    #into the calculations as current position of the leader
                    rob1_goal_x = x1
                    rob1_goal_y = y1
                    rob1_goal_x_px = x1
                    rob1_goal_y_px = y1
                    shape_corner_robots[0]=[rob1_goal_x_px,rob1_goal_y_px]
                    nearest_two_neighbors=find_nearest_two_neighbors(R.ID-1)
                    followers_routine_step1(R.ID-1,nearest_two_neighbors[0],nearest_two_neighbors[1])

                    next_goal_list=list()
                    next_goal_list=numpy.reshape(next_goal,(8))
                    next_goal_list[(R.ID-1)]= rob1_goal_x
                    next_goal_list[(R.ID)]= rob1_goal_y
                    pub_update_next_goals.publish(Int32MultiArray(data= next_goal_list ))

                    leader_calc_flag = True
                leaderGoalFlag.publish(1)


        elif (R.status == 2):#follower1
            rospy.Subscriber('leader_reached_flag', Byte, leader_flag_callback)
            follower1GoalFlag = rospy.Publisher('follower1_reached_flag', Byte, queue_size=10)
            connections = follower1GoalFlag.get_num_connections()
            if connections > 0:
                follower1GoalFlag.publish(-1)
            while not leader_goal_flag :
                print "waiting for leader"
                time.sleep (5)
            global x2 ,y2,rob2_goal_x ,rob2_goal_y,rob2_goal_x_px ,rob2_goal_y_px
            nearest_not_aligned_neighbor = find_nearest_not_aligned_neighbor(R.ID-1)
            follower_routine_step2(R.ID-1,nearest_not_aligned_neighbor)
            move(R.ID)
            follower1GoalFlag.publish(1)

        elif (R.status == 3):#follower2
            rospy.Subscriber('follower1_reached_flag', Byte, follower1_flag_callback)
            while not follower1_goal_flag :
                print "waiting for leader"
                time.sleep (12)
            global x3 ,y3,rob3_goal_x_px ,rob3_goal_y_px
            nearest_not_aligned_neighbor = find_nearest_not_aligned_neighbor(R.ID-1)
            follower_routine_step2(R.ID-1,nearest_not_aligned_neighbor)
            move(R.ID)
            follower2GoalFlag.publish(1)

    else:
        print 'waiting required_shape'

###############################################################################
#followers_routine_step1 function:
###############################################################################
def followers_routine_step1(leader_id,f1_id,f2_id):
    ''' this function do the routine for each follower from leader prespective
    '''
    # pub_ids = rospy.Publisher('leader_followers_ids',Int32MultiArray,queue_size=10)
    # pub_ids.publish(Int32MultiArray(data=numpy.array([leader_id , f1_id , f2_id],Int32MultiArray))
    # rospy.Subscriber('leader_followers_distances',Int32MultiArray, callback_leader_follower_distances )
    global shape_sides
    leader_follower1_x = poses[leader_id][0] - poses[f1_id][0]
    leader_follower1_y = poses[leader_id][1] - poses[f1_id][1]
    leader_follower2_x = poses[leader_id][0] - poses[f2_id][0]
    leader_follower2_y = poses[leader_id][1] - poses[f2_id][1]
    #follower1 procedure:
    if ((leader_follower1_x < leader_follower1_y) and align_axis[leader_id][0]==0):
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'x' )
    else:
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'y' )
    shape_sides -= 1
    shape_corner_robots[f1_id]=next_goal[f1_id]
    #follower2 procedure:
    if ((leader_follower2_x < leader_follower2_y) and align_axis[leader_id][0]==0):
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'x' )
    else:
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'y' )
    shape_sides -= 1
    shape_corner_robots[f2_id]=next_goal[f2_id]

###############################################################################
#followers_routine_step2 function:
###############################################################################
def followers_routine_step2(follower_id,neighbor_id):
    ''' this function do the routine for each follower from follower itself
        to its neighbor
    '''
    follower_neighbor_x = poses[follower_id][0] - poses[neighbor_id][0]
    follower_neighbor_y = poses[follower_id][1] - poses[neighbor_id][1]
    #procedure:
    if ((follower_neighbor_x < follower_neighbor_y) and align_axis[follower_id][0]==0):
        align(poses[neighbor_id], neighbor_id ,poses[follower_id],leader_id ,'x' )
    else:
        align(poses[neighbor_id], neighbor_id ,poses[follower_id],leader_id ,'y' )
    shape_sides -= 1
    shape_corner_robots[neighbor_id] = next_goal[neighbor_id]

###############################################################################
#find_nearest_two_neighbors function:
###############################################################################
def find_nearest_two_neighbors(id):
    ''' this function return the id of the nearest two nearest two neighbors.
        we want to calc the distance between the robot which is given its id and
        the other 3  robots .. so we increase 1,2,3,... to the id and take the
        reminder so if the id  given is 2 >> the results will become 3,0,1 which
        are the other three robots ids and so on
    '''
    global poses, all_status

    dist1 = calculate_distance(poses[id],poses[(id+1)%4])
    dist2 = calculate_distance(poses[id],poses[(id+2)%4])
    dist3 = calculate_distance(poses[id],poses[(id+3)%4])
    distances ={str((id+1)%4):dist1,  str((id+2)%4):dist2,  str((id+3)%4):dist3}
    # sort the dictionary by values:
    sorted_distances = sorted(distances.iteritems(), key = lambda x : x[1])
    # make a list of the first 2 keys as integers to be returned:
    nearest_two_neighbors_list= [int(sorted_distances[0][0]),int(sorted_distances[1][0])]
    l=list()
    l= all_status
    l[nearest_two_neighbors_list[0]] = 2
    l[nearest_two_neighbors_list[1]] = 3
    pub_assign_follower_robots. publish( Int32MultiArray(data=l) )
    return nearest_two_neighbors_list

###############################################################################
#find_nearest_not_aligned_neighbor function:
###############################################################################
def find_nearest_not_aligned_neighbor(id):
    ''' this function return the id of the not not_aligned_neighbors to the
    given robot id.
    we want to calc the distance between the robot which is given its id and
    the other 3  robots .. so we increase 1,2,3,... to the id and take the
    reminder so if the id  given is 2 >> the results will become 3,0,1 which
    are the other three robots ids and so on
    '''
    dist1 = calculate_distance(poses[id],poses[(id+1)%4])
    dist2 = calculate_distance(poses[id],poses[(id+2)%4])
    dist3 = calculate_distance(poses[id],poses[(id+3)%4])
    distances ={str((id+1)%4):dist1,  str((id+2)%4):dist2,  str((id+3)%4):dist3}
    # sort the dictionary by values:
    sorted_distances = sorted(distances.iteritems(), key = lambda x : x[1])
    # make a list of the first 1 keys as integers to be returned:
    key1 = int(sorted_distances[0][0])
    key2 = int(sorted_distances[1][0])
    key3 = int(sorted_distances[2][0])
    nearest_not_aligned_neighbor=key1
    for k in [key1,key2,key3]:
        if(align_axis[k][0]!=1 and align_axis[k][1]!=1):
            #then it is not aligned
            robotnearest_not_aligned_neighbor = k

    return nearest_not_aligned_neighbor

###############################################################################
#calculate_distance function:
###############################################################################
def calculate_distance(p1,p2):
    dist= math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))
    return dist

###############################################################################
#move function:
###############################################################################
def move(id):
    ''' rise flag with the id of the robot to be moved
    '''
    pub_robot_id=Publisher('robot_to_be_moved',Int32,queue_size=10)
    pub_robot_id.publish(id)
    while g2g_flag !=1:
        print "do nothing and wait"
        rospy.sleep(150)

###############################################################################
#Main:
###############################################################################
if __name__== '__main__':
    rospy.init_node('Form')
    while not rospy.is_shutdown():
        who_am_I()
        listener_from_global()
        listener()
        #	rate.sleep()
        rospy.spin()
