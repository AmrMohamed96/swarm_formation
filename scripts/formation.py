#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, Int32, String, Byte
import numpy
import math
import time

###############################################################################
# 2018 FORMATION VARIABLES
###############################################################################
#desired orientation difference:
Ad2 =Ad3 =Ad4= 0
#desired distance of the formation shape
Dd=0
#orientation in radians:
A2=A3=0
#formation shape:
shapes = ''

###############################################################################
# CONSTANTS FOR THE FORMATION 2019
###############################################################################
c2c_distance_px=1 # it is published from the user input and if not it by default =1
grid_dim=17.5 #cm
grid_dig_dim=24.75 #cm
swarm_robots_num=4 # this year we have 4 robots

###############################################################################
# VARIABLES THAT ARE LATER CHANGED WITHIN CODE - ASSIGNED HERE TO BE GLOBAL
###############################################################################
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
#incrementation values: (needed for the old calculations function)
m=n=0

###############################################################################
# DECENTERALIZED FORMATION VARIABLES 2019
###############################################################################
#all_status initialization: (array to be published different from the recieved one)
all_status=[0,0,0,0]
#all robots positions : (array to be subscribed from the cam.)
poses=[[0,0],[0,0],[0,0],[0,0]]
# calculations flag
leader_calc_flag = False
#next_goal array:
next_goal=[[0,0],[0,0],[0,0],[0,0]]
next_goal_list=[[0,0],[0,0],[0,0],[0,0]]
align_axis=[[0,0],[0,0],[0,0],[0,0]]
shape_corner_robots=[[0,0],[0,0],[0,0],[0,0]]

# Robot Finish Flags Initialization
leader_goal_flag = 0
follower1_goal_flag = 0
follower2_goal_flag = 0
g2g_flag = 0

# VARIABLES TO ACCOUNT  FOR LATENCY AND OPTIMIZE CODE RUNNING
system_latency = 5
final_finish_flag = 0

possible_flag = 0

###############################################################################
# ROBOT CLASS - CONTAINS ROBOT RELATED PARAMETERS
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
# PUBLISHERS DECLARATIONS
###############################################################################
# robot goals in px - used for path planning
pub_rob1_goal_px = rospy.Publisher('robot1_goal_px', Int32MultiArray, queue_size=10)
pub_rob2_goal_px = rospy.Publisher('robot2_goal_px', Int32MultiArray, queue_size=10)
pub_rob3_goal_px = rospy.Publisher('robot3_goal_px', Int32MultiArray, queue_size=10)
pub_rob4_goal_px = rospy.Publisher('robot4_goal_px', Int32MultiArray, queue_size=10)

# robot goals in cm
pub_rob1_goal_cm = rospy.Publisher('robot1_goal_cm',Int32MultiArray,queue_size=10)
pub_rob2_goal_cm = rospy.Publisher('robot2_goal_cm', Int32MultiArray, queue_size=10)
pub_rob3_goal_cm = rospy.Publisher('robot3_goal_cm', Int32MultiArray, queue_size=10)
pub_rob4_goal_cm = rospy.Publisher('robot4_goal_cm', Int32MultiArray, queue_size=10)

# publisher to update follower robot's status in robot_status global array
pub_assign_follower_robots = rospy.Publisher('assign_follower_robots', Int32MultiArray, latch=True, queue_size=10)

# publisher to update the next_goals array of the global node
pub_update_next_goals = rospy.Publisher('update_next_goals',Int32MultiArray, latch=True, queue_size=10, )

# 2018 - OLD CALCULATION PUBLISHER
pub_m_n_values = rospy.Publisher('m_n_values', Int32MultiArray, latch=True, queue_size=10)

# path planning Robot To Move Flag publisher
pub_robot_id= rospy.Publisher('robot_to_be_moved',Int32, queue_size=10)

###############################################################################
# CALLBACK FUNCTIONS FOR ROS SUBSCRIBERS
###############################################################################
def callback_status (data):
    global R, all_status
    R.set_status(data.data[R.ID-1])
    all_status=[]
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

def callback_x_sides(data): #needed for saturation secinarios
    global x_sides
    x_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_y_sides(data): #needed for saturation secinarios
    global y_sides
    y_sides= [data.data[0],data.data[1],data.data[2],data.data[3]]

def callback_shape_sides(data):
    global shape_sides
    shape_sides = data.data

def callback_shape_length(data):
    global shape_length, c2c_distance_px
    shape_length = data.data
    c2c_distance_px = int (shape_length // grid_dim)

def callback_g2g_flag_rob (data):
    global g2g_flag
    g2g_flag=data.data

def callback_formation_shape(data): # formation shape:
    global shapes
    shapes = data.data

def callback_rob1_current_pos(data): #current position of robot1
    global x1, y1, a1
    #rospy.loginfo('robot1 current position and rotation are {}'.format(data.data))
    x1 = data.data[0]
    y1 = data.data[1]
    a1 = data.data[2]

def callback_rob2_current_pos(data): #current position of robot 2
    global x2, y2, a2
    #rospy.loginfo('robot2 current position and rotation are {}'.format(data.data))
    x2 = data.data[0]
    y2 = data.data[1]
    a2 = data.data[2]

def callback_rob3_current_pos(data): #current position of robot 3
    global x3, y3, a3
    #rospy.loginfo('robot3 current position and rotation are {}'.format(data.data))
    x3 = data.data[0]
    y3 = data.data[1]
    a3 = data.data[2]

def callback_rob4_current_pos(data): #current position of robot 4
    global x4, y4, a3
    #rospy.loginfo('robot4 current position and rotation are {}'.format(data.data))
    x4 = data.data[0]
    y4 = data.data[1]
    a4 = data.data[2]
    #call the shape and final functions after subscription of all robots positions
    shape()
    final()

def callback_robots_current_poses(data):
    ''' this call back fn takes all current poses from camera direct in one array
    '''
    global poses
    poses = numpy.reshape(data.data, (4,2))
    # for i in range(0,4):
    #     rospy.loginfo('robot{} current position is {} '.format(i+1,poses[i]))
    # print"_____________________________________________________________________"

def callback_leader_final_goal(data):
    global R1_gx_cm, R1_gy_cm
    rospy.loginfo('leader will move to new goals {}'.format(data.data))
    R1_gx_cm = data.data[0]
    R1_gy_cm = data.data[1]

def possibility_callback(data):
    global possible_flag
    possible_flag = data.data

###############################################################################
# WHO AM I FUNCTION - IT RUNS ONCE WITHIN MAIN CODE
###############################################################################
def who_am_I():
    '''
    This function creates a robot class with an ID defined in the launch file for the robot
    It also shows his role (L, F1, F2, F3) as stated in the robot_status global list
    '''
    global R

    R = RobotClass()

    # creating publisher and subscriber for each robot based on it's id
    # as this code will run on all 4 robots
    pub_corners_rob = rospy.Publisher('corners_rob'+str(R.ID), Int32MultiArray,queue_size=10)

    rospy.loginfo('Robot [%d] class created. Waiting some time to ensure connections to robot_status are established'%(R.ID))
    # some wait time to ensure subscriber has actually made a connection
    time.sleep(system_latency)

    rospy.loginfo('I am Robot [%d]. My Current Status is [%d]' % (R.ID,R.status))

###############################################################################
# ROS SUBSCRIBERS
###############################################################################
def listeners():
    # SUBSCRIPTIONS FROM GLOBAL NODE
    rospy.Subscriber('formation_shape_name', String, callback_formation_shape)
    rospy.Subscriber('shape_sides',Int32,callback_shape_sides)
    rospy.Subscriber('shape_length',Int32,callback_shape_length)
    rospy.Subscriber('x_sides',Int32MultiArray,callback_x_sides)
    rospy.Subscriber('y_sides',Int32MultiArray,callback_y_sides)

    rospy.Subscriber('align_axis',Int32MultiArray,callback_align_axis)
    rospy.Subscriber('shape_corner_robots',Int32MultiArray,callback_shape_corner_robots)
    rospy.Subscriber('next_goals',Int32MultiArray,callback_next_goal)

    # SUBSCRIPTIONS FROM CAMERA LOCALIZATION NODE
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback_rob1_current_pos)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, callback_rob2_current_pos)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, callback_rob3_current_pos)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, callback_rob4_current_pos)
    rospy.Subscriber('robots_current_poses', Int32MultiArray, callback_robots_current_poses)

    # GO TO GOAL FLAGS SUBSCRIPTIONS
    rospy.Subscriber('leader_final_goal', Int32MultiArray, callback_leader_final_goal)
    rospy.Subscriber('gtg_flag_rob'+str(R.ID), Int32, callback_g2g_flag_rob)

    # robot status subscriber
    rospy.Subscriber('robot_status', Int32MultiArray, callback_status)

    # possibility flag
    rospy.Subscriber('formation_possible_flag', Byte, possibility_callback)

###############################################################################
# JUSTIFY DISTANCE FUNCTION
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
    global next_goal, align_axis
    #1. initialize justified Point to 0,0:
    justified_point=[0,0]

    # flag of 3 means any quad will be okay
    if possible_flag == 3:
        #2. check that next and bas sharing the same x:
        if next[0]==base[0]:
            justified_point[0]= base[0]
            if base[1]<next[1]: #next is above the base
                justified_point[1]=base[1] + length
            else: #next is below
                justified_point[1]=base[1] - length

        #3. check that next and bas sharing the same y:
        elif next[1]==base[1]:
            justified_point[1]= base[1]
            if base[0]<next[0]: #next is above the base
                justified_point[0]=base[0] + length
            else:
                justified_point[0]=base[0] - length

    # flag of 2 means only 4th quad case
    if possible_flag == 2:
        #2. check that next and bas sharing the same x:
        if next[0]==base[0]:
            justified_point[0]= base[0]
            justified_point[1]= base[1]-length

        #3. check that next and bas sharing the same y:
        elif next[1]==base[1]:
            justified_point[1]= base[1]
            justified_point[0]=base[0]-length

    # flag of 1 means only 1st quad case
    if possible_flag == 1:
        #2. check that next and bas sharing the same x:
        if next[0]==base[0]:
            justified_point[0]= base[0]
            justified_point[1]= base[1] + length

        #3. check that next and bas sharing the same y:
        elif next[1]==base[1]:
            justified_point[1]= base[1]
            justified_point[0]= base[0] + length

    # flag of zero means we need to set another leader
    if possible_flag == 0:
        rospy.logerr('Formation will not be possible. Reset leader or restart system')
        while 1:
            continue

    return justified_point

###############################################################################
# ALIGN FUNCTION
###############################################################################
def align(follower,follower_num,leader,leader_num,direction):
    ''' this function determine the goal position for follower based on
        alignment direction.The robot align in 90 degree in x or y direction.
        The function takes direction,follower and leader position and
        determines the next goal according to direction specified. The resulted
        goal position is then modified according to shape length by calling
        justify_distance function. align_axis global flags are updated after goal
        position is chosen for both follower and leader
    '''
    global next_goal, align_axis, poses
    next_list=next_goal
    #align in x or y as demand:
    if (direction=='x'):
        rospy.loginfo('Aligning Robot [{}] with Robot [{}] in [X] Axis'.format(follower_num+1, leader_num+1))
        #1. make the next goal in x equals to follower x :
        next_list[follower_num][0]=follower[0]
        #2. make the next goal in y equals to leader y :
        next_list[follower_num][1]=leader[1]
        #3. set the align axis:
        align_axis_leader = align_axis[leader_num]
        align_axis_follower = align_axis[follower_num]
        align_axis_leader[0]=1
        align_axis_follower[0]=1
        rospy.loginfo('Align Axis for Robot [{}] is now: {}'.format(leader_num,align_axis_leader))

        pub_align_follower = rospy.Publisher('align_axis_rob'+str(follower_num+1), Int32MultiArray, queue_size=10)
        pub_align_rob = rospy.Publisher('align_axis_rob'+str(R.ID), Int32MultiArray,queue_size=10)
        rospy.loginfo('Sleeping some time to ensure connections to align axis')
        time.sleep(5)
        pub_align_rob.publish( Int32MultiArray(data=align_axis_leader))
        pub_align_follower.publish( Int32MultiArray(data=align_axis_follower))


    elif (direction=='y'):
        rospy.loginfo('Aligning Robot [{}] with Robot [{}] in [Y] Axis'.format(follower_num+1, leader_num+1))
        #1. make the next goal in x equals to leader x :
        next_list[follower_num][0]=leader[0]
        #2. make the next goal in y equals to follower y :
        next_list[follower_num][1]=follower[1]
        #3. set the align axis:
        align_axis_leader = align_axis[leader_num]
        align_axis_follower = align_axis[follower_num]
        align_axis_leader[1]=1
        align_axis_follower[1]=1
        rospy.loginfo('Align Axis for Robot [{}] is now: {}'.format(leader_num,align_axis_leader))

        pub_align_follower = rospy.Publisher('align_axis_rob'+str(follower_num+1), Int32MultiArray ,queue_size=10)
        pub_align_rob = rospy.Publisher('align_axis_rob'+str(R.ID), Int32MultiArray,queue_size=10)
        rospy.loginfo('Sleeping some time to ensure connections to align axis')
        time.sleep(5)
        pub_align_rob.publish( Int32MultiArray(data=align_axis_leader))
        pub_align_follower.publish( Int32MultiArray(data=align_axis_follower))

    #update next_goal of the follower:
    next_list[follower_num] = justify_distance(next_list[follower_num],leader,shape_length)
    next_list=numpy.reshape(next_list,(8))
    pub_update_next_goals.publish(Int32MultiArray(data= next_list))
    rospy.loginfo('Sleeping some time to ensure other robots get the updated data in next_goals')
    time.sleep(5)


###############################################################################
#shape function:
###############################################################################
def shape():
    '''
     2018-
     this function checks what formation is sent and set
     the orientations and distance between the robots desired.
     also set the incrementations the leader should move if obstacles found
    '''
    global Ad2, Ad3, Ad4, Dd
    global shapes, m, n, c2c_distance_px, grid_dim ,grid_dig_dim
    if shapes == "line" :
        Ad2=0
        Ad3=180
        Dd =swarm_robots_num* c2c_distance_px*grid_dim
        Ad4=0
        m=2
        n=0
    elif shapes == "column" :
        Ad2=90
        Ad3=270
        Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=0
        n=2
    elif shapes == "diagonal" :
        Ad2=45
        Ad3=-135
        Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=2
        n=1
    elif shapes == "triangle":
        Ad2=-45
        Ad3=-135
        Ad4=270
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=2
        n=1
    elif shapes == "L-shape":
        Ad2=0
        Ad3=90
        Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    elif shapes == "square":
        Ad2=0
        Ad3=90
        Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    #publish the incrementation values of leader:
    pub_m_n_values.publish( Int32MultiArray(data=[m,n]) )

###############################################################################
# 2018 FORMATION CALCULATION FUNCTION
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
# 2019 FORMATION FUNCTIONS
###############################################################################
def leader_flag_callback(data):
    global leader_goal_flag
    leader_goal_flag = data.data

def follower1_flag_callback(data):
    global follower1_goal_flag
    follower1_goal_flag = data.data

def follower2_flag_callback(data):
    global follower2_goal_flag
    follower2_goal_flag = data.data

leader_finish_flag = 0

def final():
    ''' this function is run after the whole robots position is subscribed and
        are ready to use in this code.
        the main purpose of this function is to check shape is entered and
        check leader goal is modified or not and calculate the follower1 and 2
        new goals and publish them.
    '''
    global next_goal, align_axis,shape_corner_robots,poses, leader_finish_flag, final_finish_flag
    #rospy.loginfo('Entered the final() function')

    if ( shapes != '' ) and ( final_finish_flag != 1):
        if (R.status==1): #leader
            global x1 ,y1,rob1_goal_x ,rob1_goal_y,rob1_goal_x_px ,rob1_goal_y_px, leader_calc_flag
            rospy.loginfo('Robot {} started the main leader function'.format(R.ID))

            # creating a leader flag publisher to raise the flag after leader has finished it's procedure
            leaderGoalFlag = rospy.Publisher('leader_reached_flag', Byte, latch=True, queue_size=10)
            leaderGoalFlag.publish(leader_finish_flag)

            if not leader_calc_flag:
                rospy.loginfo('Doing leader-based calculations ... 1st RUN')

                #if leader has a new position set the position to be added
                #into the calculations as the new position
                if ((R1_gx_cm > 0) and (R1_gy_cm > 0)):
                    rospy.loginfo('Leader needs to move. Will make calculations')
                    time.sleep(system_latency)

                    next_goal_list = next_goal #old next_goal list of lists that we subscribed
                    next_goal_list=numpy.reshape(next_goal,(8))
                    my_index = ( 2 * R.ID ) - 2
                    next_goal_list[my_index]= R1_gx_cm
                    next_goal_list[my_index + 1]= R1_gy_cm

                    pub_update_next_goals.publish(Int32MultiArray(data= next_goal_list ))

                    shape_corner_robots[0]=[rob1_goal_x,rob1_goal_y]
                    nearest_two_neighbors=find_nearest_two_neighbors(R.ID-1)
                    followers_routine_step1(R.ID-1,nearest_two_neighbors[0],nearest_two_neighbors[1])

                    move(R.ID)
                    leader_calc_flag = True
                    leader_finish_flag = 1

                else:
                    rospy.loginfo('Leader does not need to move. Calculating for followers')
                    time.sleep(system_latency)

                    #if leader is not supposed to move set the positions to be added
                    #into the calculations as current position of the leader

                    next_goal_list = next_goal
                    next_goal_list = numpy.reshape(next_goal,(8))
                    my_index = ( 2 * R.ID ) - 2
                    next_goal_list[my_index] = poses[R.ID-1][0]
                    next_goal_list[my_index + 1] = poses[R.ID-1][1]

                    pub_update_next_goals.publish( Int32MultiArray(data=next_goal_list) )

                    shape_corner_robots[0]=[rob1_goal_x,rob1_goal_y]
                    nearest_two_neighbors=find_nearest_two_neighbors(R.ID-1)
                    followers_routine_step1(R.ID-1,nearest_two_neighbors[0],nearest_two_neighbors[1])

                    leader_calc_flag = True
                    leader_finish_flag = 1

                leaderGoalFlag.publish(leader_finish_flag)
                final_finish_flag = 1
                print "Everything was done. Spinning"

        elif (R.status == 2):#follower1
            rospy.loginfo('Robot {} status is FOLLOWER 1'.format(R.ID))

            # subscribe to the leader finish flag
            rospy.Subscriber('leader_reached_flag', Byte, leader_flag_callback)

            # create a publisher for follower 1 finish flag
            follower1GoalFlag = rospy.Publisher('follower1_reached_flag', Byte, latch=True, queue_size=10)

            # poll the flag until it's raised
            rospy.loginfo('Robot {} is waiting for leader to raise the finish flag'.format(R.ID))
            while not leader_goal_flag :
                pass

            global x2 ,y2,rob2_goal_x ,rob2_goal_y,rob2_goal_x_px ,rob2_goal_y_px
            nearest_not_aligned_neighbor = find_nearest_not_aligned_neighbor(R.ID-1)
            
            next_follower_id = -1

            # get the index of follower 2
            for i in range( len(all_status) ):
                if all_status[i] == 3:
                    next_follower_id = i

            followers_routine_step2(R.ID-1,nearest_not_aligned_neighbor, next_follower_id)

            move(R.ID)
            follower1GoalFlag.publish(1)
            final_finish_flag = 1
            print "Everything was done. Spinning"

        elif (R.status == 3):#follower2
            rospy.loginfo('Robot {} status is FOLLOWER 2'.format(R.ID))

            # subscribe to the follower 1 finish flag
            rospy.Subscriber('follower1_reached_flag', Byte, follower1_flag_callback)

            # create a publisher for follower 2 finish flag
            follower2GoalFlag = rospy.Publisher('follower2_reached_flag', Byte, latch = True ,queue_size=10)

            # poll the flag until it's raised
            rospy.loginfo('Robot {} is waiting for follower 1 to raise the finish flag'.format(R.ID))
            while not follower1_goal_flag :
                pass

            global x3 ,y3,rob3_goal_x_px ,rob3_goal_y_px
            nearest_not_aligned_neighbor = find_nearest_not_aligned_neighbor(R.ID-1)
            next_follower_id = -1
            # get the index of follower 2
            for i in range( len(all_status) ):
                if all_status[i] == 3:
                    next_follower_id = i

            followers_routine_step2(R.ID-1,nearest_not_aligned_neighbor,next_follower_id)

            move(R.ID)
            follower2GoalFlag.publish(1)
            final_finish_flag = 1
            print "Everything was done. Spinning"

        elif (R.status==4):
            rospy.loginfo('Robot {} status is FOLLOWER 3'.format(R.ID))

            # subscribe to the follower 2 finish flag
            rospy.Subscriber('follower2_reached_flag', Byte, follower2_flag_callback)

            # create a publisher for follower 3 finish flag
            follower3GoalFlag = rospy.Publisher('follower3_reached_flag', Byte, latch=True, queue_size=10)

            # poll the flag until it's raised
            rospy.loginfo('Robot [%d] is waiting for follower 2 to raise the finish flag'% (R.ID))
            while not follower2_goal_flag :
                pass

            move(R.ID)
            follower3GoalFlag.publish(1)
            final_finish_flag = 1
            print "Formation completed"
            print "Thank you for using SWARM 2019 Formation Algorithm"


###############################################################################
# FOLLOWER 1 CALCULATIONS ROUTINE
###############################################################################
def followers_routine_step1(leader_id,f1_id,f2_id):
    ''' this function do the routine for each follower from leader prespective
    '''
    global shape_sides, next_goal,next_goal_list,poses
    rospy.loginfo('Started FOLLOWER 1 ROUTINE')

    leader_follower1_x = abs( poses[leader_id][0] - poses[f1_id][0] )
    leader_follower1_y = abs( poses[leader_id][1] - poses[f1_id][1] )

    leader_follower2_x = abs( poses[leader_id][0] - poses[f2_id][0] )
    leader_follower2_y = abs( poses[leader_id][1] - poses[f2_id][1] )

    #follower1 procedure:
    if ((leader_follower1_x < leader_follower1_y) and align_axis[leader_id][1]==0):
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'y' )
    elif ((leader_follower1_x > leader_follower1_y) and align_axis[leader_id][0]==0):
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'x' )
    elif align_axis[leader_id][0]==0:
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'x' )
    elif align_axis[leader_id][1]==0:
        align(poses[f1_id], f1_id ,poses[leader_id],leader_id ,'y' )

    shape_sides -= 1
    shape_corner_robots[f1_id]=next_goal[f1_id]

    #follower2 procedure:
    if ((leader_follower2_x < leader_follower2_y) and align_axis[leader_id][1]==0):
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'y' )
    elif ((leader_follower2_x > leader_follower2_y) and align_axis[leader_id][0]==0):
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'x' )
    elif align_axis[leader_id][0]==0:
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'x' )
    elif align_axis[leader_id][1]==0:
        align(poses[f2_id], f2_id ,poses[leader_id],leader_id ,'y' )

    shape_sides -= 1
    shape_corner_robots[f2_id]=next_goal[f2_id]

###############################################################################
# FOLLOWER 2 CALCULATIONS ROUTINE
###############################################################################
def followers_routine_step2(follower_id,neighbor_id,next_follower_id):
    ''' this function do the routine for each follower from follower itself
        to its neighbor
    '''
    global shape_sides
    if next_follower_id == follower_id:
        pass
    else:
        # Do the distance calculations
        distf2 = calculate_distance(poses[next_follower_id], poses[neighbor_id])

        distf1 = calculate_distance(poses[follower_id], poses[neighbor_id])

        if distf2 < distf1:
            return 
        else:
            pass

    if (neighbor_id != 55):
        rospy.loginfo('Started FOLLOWER 2 ROUTINE')

        follower_neighbor_x = abs( next_goal[follower_id][0] - poses[neighbor_id][0] )
        follower_neighbor_y = abs( next_goal[follower_id][1] - poses[neighbor_id][1] )

        #procedure:
        if ((follower_neighbor_x < follower_neighbor_y) or align_axis[follower_id][0]==0):
            align(poses[neighbor_id], neighbor_id ,next_goal[follower_id],follower_id ,'x' )

        elif((follower_neighbor_x > follower_neighbor_y) or align_axis[follower_id][1]==1):
            align(poses[neighbor_id], neighbor_id ,next_goal[follower_id],follower_id ,'y' )

        shape_sides -= 1
        shape_corner_robots[neighbor_id] = next_goal[neighbor_id]
    else:
        pass

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
    rospy.loginfo('Calculating nearest two neighbours for Robot [%d]' %(id+1))

    dist1 = calculate_distance(poses[id],poses[(id+1)%4])
    dist2 = calculate_distance(poses[id],poses[(id+2)%4])
    dist3 = calculate_distance(poses[id],poses[(id+3)%4])
    distances ={str((id+1)%4):dist1,  str((id+2)%4):dist2,  str((id+3)%4):dist3}

    # sort the dictionary by values:
    sorted_distances = sorted(distances.iteritems(), key = lambda x : x[1])

    # make a list of the first 2 keys as integers to be returned:
    nearest_two_neighbors_list= [int(sorted_distances[0][0]),int(sorted_distances[1][0])]

    rospy.loginfo('Nearest two neighbours are Robot [%d], Robot [%d]' %( nearest_two_neighbors_list[0]+1 , nearest_two_neighbors_list[1]+1))

    update_all_status = []
    update_all_status = all_status
    update_all_status[nearest_two_neighbors_list[0]] = 2
    update_all_status[nearest_two_neighbors_list[1]] = 3
    pub_assign_follower_robots.publish( Int32MultiArray(data=update_all_status))

    return nearest_two_neighbors_list

###############################################################################
# FINDING NEAREST NEIGHBOURS THAT ARE NOT ALIGNED
###############################################################################
def find_nearest_not_aligned_neighbor(id):
    ''' this function return the id of the not not_aligned_neighbors to the
    given robot id.
    we want to calc the distance between the robot which is given its id and
    the other 3  robots .. so we increase 1,2,3,... to the id and take the
    reminder so if the id  given is 2 >> the results will become 3,0,1 which
    are the other three robots ids and so on
    '''
    global robot_nearest_not_aligned_neighbor_id
    rospy.loginfo('Calculating nearest two unaligned neighbours for Robot [%d]' %(id+1))

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

    robot_nearest_not_aligned_neighbor_id = 55

    for k in [key1,key2,key3]:
        if(align_axis[k][0]!=1 and align_axis[k][1]!=1):
            #then it is not aligned
            robot_nearest_not_aligned_neighbor_id = k

    rospy.loginfo('Nearest Unaligned Neighbour to Robot [%d] is Robot [%d]' %(id+1,robot_nearest_not_aligned_neighbor_id+1))
    try:
        update_all_status = []
        update_all_status = all_status
        update_all_status[robot_nearest_not_aligned_neighbor_id] = 4 #follower3

        pub_assign_follower_robots.publish( Int32MultiArray(data=update_all_status) )

        return robot_nearest_not_aligned_neighbor_id
    except IndexError:
        rospy.loginfo('All robots are already aligned. Skipping Unaligned Neighbor Step')
        return 55

###############################################################################
# CALCULATING EUCLEDIAN DISTANCE
###############################################################################
def calculate_distance(p1,p2):
    """
    Function returns eucledian distance between two points
    """
    dist= math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))
    return dist

###############################################################################
# SIGNALING ROBOT MOVE FUNCTION
###############################################################################
def move(id):
    '''
    Raise a flag with the id of the robot to be moved - used for path planning
    '''
    global g2g_flag

    rospy.loginfo('Robot to be moved is: Robot [%d]' %(id))
    pub_robot_id.publish(id)

    rospy.loginfo('Waiting for the GTG Flag.')
    while g2g_flag !=1:
        pass

###############################################################################
# POSITION CHECK FUNCTION
###############################################################################
def check_positions():
    """
    Checks if robot positions are being sent
    """
    return (x1 and y1) or (x2 and y2) or (x3 and y3) or (x4 and y4)

###############################################################################
# MAIN FUNCTION
###############################################################################
if __name__== '__main__':
    rospy.init_node('formation_node_rob1')
    rospy.loginfo ('Formation node started for Robot')

    who_am_I()
    listeners()

    rospy.loginfo('All subscribers created. Waiting for some time to ensure connections are made.')
    time.sleep(system_latency)

    check = check_positions()
    if not check:
        rospy.logwarn('Robot Positions are missing')

    while not rospy.is_shutdown():
        rospy.spin()
