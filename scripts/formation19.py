#!/usr/bin/env python

import argparse
import sys
import rospy
from std_msgs.msg import Int32MultiArray Int32 String Float32 Byte
from rospy_tutorials.msg import Floats
import numpy
import math
import os
###############################################################################
#initialize all variables:
###############################################################################
#robots numbers:
leader_num=0
follower1_num=1
follower1_num=2
follower1_num=3
#desired orientation difference:
Ad2 = 0
Ad3 = 0
y=0
#desired distance:
Dd=0 #desired distance of the shape
c2c_distance_px=1 # it is published from the user input and if not it by default =1
grid_dim=17.5 #cm
grid_dig_dim=24.75 #cm
swarm_robots_num=4 # this year we have 4 robots
#orientation in radians:
A2=0
A3=0
#formation shape:
shapes = ''

#leader new position after check in cm:
R1_gx_cm = 0
R1_gy_cm =0
#leader final position
rob1_goal_x =0
rob1_goal_y =0
#leader current position:
x1=0
y1=0
a1=0
#robot 2 current position:
x2=0
y2=0
#robot 3 current position:
x3=0
y3=0
#robot 4 current position:
x4=0
y4=0
#incrementation values:
m=0
n=0
#who_am_I flag:
global flag=0
###############################################################################
#RobotClass:
###############################################################################
class RobotClass:
    ID= -1
    IP=''
    status=''
    current_goal= [[0,0],[0,0],[0,0],[0,0]]
    next_goal= [[0,0],[0,0],[0,0],[0,0]]
    def __init__(self,ID,status):
        self.ID = rospy.rosparam('~robot_id', 1)
        self.status=status
    def set_ID(self, ID):
        self.ID = ID
    def set_robot_IP(self, IP):
        self.IP = IP
    def set_status(self, status):
        self.status =status
    def set_current_goal(self, cgoal):
        self.current_goal[self.ID] = cgoal
    def set_next_goal(self, ngoal):
        self.next_goal[self.ID] = ngoal
    def get_ID(self):
        return self.ID
    def get_IP(self):
        return self.IP
    def get_status(self):
        return self.status
    def get_next_goal(self):
        return self.next_goal[self.ID]
    def get_current_goal(self):
        return self.current_goal[self.ID]

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
pub_m_n_values = rospy.Publisher('m_n_values', Int32MultiArray, queue_size=10)
###############################################################################
#callback functions:
###############################################################################
def callback_ID (data):
    global Robot_ID
    Robot_ID=data.data

def callback_satus (data):
    global Robot_status
    Robot_status=data.data
    who_am_I()

def callback_align_axis(data):
    global align_axis
    align_axis=list()
    align_axis.append([data.data.[0],data.data[1])
    align_axis.append([data.data.[2],data.data[3])
    align_axis.append([data.data.[4],data.data[7])
    align_axis.append([data.data.[6],data.data[8])

def callback_shape_corner_robots(data):
    global shape_corner_robots
    shape_corner_robots=list()
    shape_corner_robots.append([data.data.[0],data.data[1])
    shape_corner_robots.append([data.data.[2],data.data[3])
    shape_corner_robots.append([data.data.[4],data.data[7])
    shape_corner_robots.append([data.data.[6],data.data[8])

def callback_next_goal(data):
    global next_goal
    next_goal =list()
    next_goal.append([data.data.[0],data.data[1])
    next_goal.append([data.data.[2],data.data[3])
    next_goal.append([data.data.[4],data.data[7])
    next_goal.append([data.data.[6],data.data[8])

def callback_x_sides(data):
    global x_sides
    x_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_y_sides(data):
    global y_sides
    y_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_current_leader(data):
    global current_leader
    current_leader = [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_shape_sides(data):
    global shape_sides
    shape_sides = data.data

def callback_shape_length(data):
    global shape_length
    shape_length = data.data

def callback1(data): # formation shape:
    global shapes
    shapes = data.data

def callback2(data): #center to center distance in pixels
    global c2c_distance_px
    c2c_distance_px = data.data

def callback3(data): #current position of robot1 (leader)
    global x1, y1, a1
    #rospy.loginfo('robot1 co. = %s', data.data)
    x1 = data.data[0]
    y1 = data.data[1]
    a1 = data.data[2]

def callback4(data): #current position of robot 2
    global x2, y2
    #rospy.loginfo('robot2 co. = %s', data.data)
    x2 = data.data[0]
    y2 = data.data[1]

def callback5(data): #current position of robot 3
    global x3, y3
    #rospy.loginfo('robot3 co. = %s', data.data)
    x3 = data.data[0]
    y3 = data.data[1]
    shape()
    final()

def callback6(data): #current position of robot 4
    global x4, y4
	#rospy.loginfo('robot4 co. = %s', data.data)
	x4 = data.data[0]
	y4 = data.data[1]

def callback7(data): # new position of the leader after check
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
    rospy.Subscriber('Robot_ID',Int32,callback_ID)
    rospy.Subscriber('Robot_status',String,callback_status)

    if  Robot_ID !=-1 and Robot_status!='':
        global R
        R=RobotClass(Robot_ID,Robot_status)
        pub_align_rob = rospy.Publisher('align_rob'+str(R.ID), Int32MultiArray,queue_size=10)
        pub_corners_rob =rospy.Publisher('corners_rob'+str(R.ID), Int32MultiArray,queue_size=10)
        print "I am robot",R.ID,"with status:",R.status
        flag=1
    else:
        flag=-1

###############################################################################
#define subscribers:
###############################################################################
def listener():
    rospy.Subscriber('Required_Shape', String, callback1)
    rospy.Subscriber('C2C_distance',Int32,callback2)
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback3)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, callback4)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, callback5)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, callback6)
    rospy.Subscriber('rob1_final_cm', Int32MultiArray, callback7)

def listener_from_global():
    rospy.Subscriber('align_axis',Int32MultiArray,callback_align_axis)
    rospy.Subscriber('shape_corner_robots',Int32MultiArray,callback_shape_corner_robots)
    rospy.Subscriber('next_goal',Int32MultiArray,callback_next_goal)
    rospy.Subscriber('x_sides',Int32MultiArray,callback_x_sides)
    rospy.Subscriber('y_sides',Int32MultiArray,callback_y_sides)
    rospy.Subscriber('current_leader',Int32MultiArray,callback_current_leader)
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
    justified_point[0]=0 # x of the point
    justified_point[1]=0 # y of the point
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
def align(follower,follower_num,leader,direction):
    ''' this function determine the goal position for follower based on alignment direction.
        The robot align in 90 degree in x or y direction. The
        function takes direction, follower and leader position anddetermine the next goal point according to direction
        specified. The resulted goal position is then modified
        according to shape length by calling a justify_distance
        function. align_axis global flags are updated after goal
        position is chosen for both follower and leader
    '''
    #align in x or y as demand:
    if (direction=='x'):
        #1. align the next_goal x index with leader x index:
        next_goal[follower_num][0]=leader[0]
        #2. leave the y index of the next_goal as the follower:
        next_goal[follower_num][1]=follower[1]
        #3. set the align axis:
        align_axis[leader_num][0]=1
        align_axis[follower_num][0]=1
    elif (direction=='y'):
        #1. align the next_goal y index with leader y index:
        next_goal[follower_num][1]=leader[1]
        #2. leave the x index of the next_goal as the follower:
        next_goal[follower_num][0]=follower[0]
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

def final():
    ''' this function is run after the whole robots position is subscribed and
        are ready to use in this code.
        the main purpose of this function is to check shape is entered and
        check leader goal is modified or not and calculate the follower1 and 2
        new goals and publish them.
    '''
    #print 'cm' ,R1_gx_cm , R1_gy_cm
    if ((shapes != '') and (flag =1)):
        if (R.status=='leader'):
            global x1 ,y1,rob1_goal_x ,rob1_goal_y,rob1_goal_x_px ,rob1_goal_y_px
            leaderGoalFlag = rospy.Publisher('leader_reached_flag', Byte, queue_size=10)
            connections = leaderGoalFlag.get_num_connection()
            if connections > 0:
                leaderGoalFlag.publish(-1)
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
                nearest_two_neighbors=find_nearest_two_neighbors()
                follower_routine()
                #calculations()
                move(R.ID,next_goal[R.ID])
            else:
                #if leader is not supposed to move set the positions to be added
                #into the calculations as current position of the leader
                rob1_goal_x = x1
                rob1_goal_y = y1
                rob1_goal_x_px = x1
                rob1_goal_y_px = y1
                shape_corner_robots[0]=[rob1_goal_x_px,rob1_goal_y_px]
                nearest_two_neighbors=find_nearest_two_neighbors()
                followers_routine_step1()
                #calculations()
                move(R.ID,next_goal[R.ID])
        elif (R.status =='follower1'):
            rospy.Subscriber('leader_reached_flag', Byte, follower1_flag_callback)
            follower1GoalFlag = rospy.Publisher('follower1_reached_flag', Byte, queue_size=10)
            connections = follower1GoalFlag.get_num_connection()
            if connections > 0:
                follower1GoalFlag.publish(-1)
            if ( leader_goal_flag == 1 ):
                global x2 ,y2,rob2_goal_x ,rob2_goal_y,rob2_goal_x_px ,rob2_goal_y_px
                shape_corner_robots[1]=[rob2_goal_x_px,rob2_goal_y_px]
                nearest_two_neighbors=find_nearest_two_neighbors()
                followers_routine_step2()
                #calculations()
                move(R.ID,next_goal[R.ID])

        elif (R.status =='follower2'):
            rospy.Subscriber('follower1_reached_flag', Byte, follower2_flag_callback)
            if ( follower1_goal_flag == 1 ):
                global x3 ,y3,rob3_goal_x_px ,rob3_goal_y_px
                shape_corner_robots[2]=[rob3_goal_x,rob3_goal_y]
                nearest_two_neighbors=find_nearest_two_neighbors()
                follower_routine()
                #calculations()
                move(R.ID,next_goal[R.ID])

        else:#tare2t el sabken le el robot el rab3 :D
            rob4_goal_x_cm = R1_gx_cm+ c2c_distance_px*grid_dim
            rob4_goal_y_cm = R1_gy_cm+ c2c_distance_px*grid_dim
            rob4_goal_x_px = math.ceil(((rob4_goal_x_cm * 2.43308) /42.9))-1
            rob4_goal_y_px = math.ceil(((rob4_goal_y_cm* 2.43308) /40.4))-1

            rob4_goal_cm = numpy.array([rob4_goal_x_cm ,rob4_goal_y_cm],Int32MultiArray)
            R4_goal_cm=Int32MultiArray(data=rob4_goal_cm)
            pub_rob4_goal_cm.publish(R4_goal_cm)
            rob4_goal_px = numpy.array([rob4_goal_x_px ,rob4_goal_y_px ],Int32MultiArray)
            R4_goal_px=Int32MultiArray(data=rob4_goal_px)
            pub_rob4_goal_px.publish(R4_goal_px)
            move(R.ID,next_goal[R.ID])

    else:
        print 'waiting required_shape'

    #change the positions into pixels and publish it
    rob1_goal_x_px = math.ceil(((rob1_goal_x * 2.43308) /42.9))-1
    rob1_goal_y_px = math.ceil(((rob1_goal_y * 2.43308) /40.4))-1
    rob1_goal_px = numpy.array([rob1_goal_x_px ,rob1_goal_y_px ],Int32MultiArray)
    R1_goal=Int32MultiArray(data=rob1_goal_px)
    #print rob1_goal_px
    pub_rob1_goal_px.publish(R1_goal)


###############################################################################
#followers_routine_step1 function:
###############################################################################
def followers_routine_step1():
    ''' this function do the routine for each follower from leader prespective
    '''


###############################################################################
#followers_routine_step2 function:
###############################################################################
def followers_routine_step2():
    ''' this function do the routine for each follower from follower itself
        to its neighbor
    '''

###############################################################################
#find_nearest_two_neighbors function:
###############################################################################
def find_nearest_two_neighbors():
    ''' this function return the id of the nearest two nearest two neighbors
    '''

###############################################################################
#move function:
###############################################################################
def move(id,goal):
    ''' this function takes the goal points and robot id to use them calling
        path planning and go to goal
    '''


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
