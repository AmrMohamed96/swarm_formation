#!/usr/bin/env python
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
from   std_msgs.msg import Int32MultiArray
from   std_msgs.msg import String,Byte
import numpy as np
import shapely
from shapely.geometry import LineString, Point
from bisect import bisect_left

line_corner1_x , square_corner1_y = 0,0
line_corner2_x , square_corner2_y = 0,0
line_corner3_x , square_corner3_y = 0,0
line_corner4_x , square_corner4_y = 0,0

r = 0

quad = 1

trail = 0

square_mp_x = 0
square_mp_y = 0
square_corner1_x = 0
square_corner1_y = 0
square_corner2_x = 0
square_corner2_y = 0
square_corner3_x = 0
square_corner3_y = 0
square_corner4_x = 0
square_corner4_y = 0
closest_corner = 1

best_leader_x = 0
best_leader_y = 0

x,y = 0,0
intx , inty = 0,0
leader_x , leader_y =0,0
follower1_x , follower1_y =0,0
follower2_x , follower2_y =0,0
follower3_x , follower3_y =0,0
mp_x=[] #midpoint x
mp_y=[] #midpoint y
min_x =[]
min_y =[]
minx , miny = 0,0
s1,s2,s3,s4 =0,0,0,0
l1,l2,l3,l4 =0,0,0,0 #lengths between robots

x1_px , y1_px= 0,0 #robot 1 pos in px
x1,y1 = 0,0        #robot 1 pos in cm

x2_px , y2_px= 0,0 #robot 2 pos in px
x2,y2 = 0,0        #robot 2 pos in cm

x3_px , y3_px= 0,0 #robot 3 pos in px
x3,y3 = 0,0        #robot 3 pos in px

x4_px , y4_px= 0,0 #robot 4 pos in px
x4,y4 = 0,0        #robot 4 pos in px

x_o1_cm , y_o1_cm = 0,0     #obstacle1 pos in cm
x_o1_px , y_o1_px = 0,0     #obstacle1 pos in px

x_o2_cm , y_o2_cm = 0,0     ##obstacle2 pos in cm
x_o2_px , y_o2_px = 0,0     ##obstacle2 pos in px

possible_flag = 0

leader_final_goal = [leader_x , leader_y]

leaderID , shapeLength = 0,0  #leader number and shape length
follower1ID , follower2ID, follower3ID =0,0,0 #follower IDs
shapeName = 'square' #square line diagonal column

pub_quad = rospy.Publisher('formation_quad',Int32, latch=True, queue_size=10)
pub_leader_final_goal = rospy.Publisher('leader_final_goal',Int32MultiArray, queue_size=10)
pub_leader_id = rospy.Publisher('set_new_leader',Byte, queue_size=10)

calculated = 0

##set the callbacks function
def callback_rob1_pose_px(data):# for pose of the robot 1 in pixel
	global x1_px , y1_px
	x1_px = data.data[0]
	y1_px = data.data[1]
def callback_rob1_CurrentPose(data):# for pose of the robot 1 in cm
	global x1,y1
	x1 = data.data[0]
	y1 = data.data[1]
####################################################
def callback_rob2_pose_px(data):# for pose of the robot 2 in pixel
	global x2_px , y2_px
	x2_px = data.data[0]
	y2_px = data.data[1]
def callback_rob2_CurrentPose(data):# for pose of the robot 2 in cm
	global x2,y2
	x2 = data.data[0]
	y2 = data.data[1]
####################################################
def callback_rob3_pose_px(data):# for pose of the robot 3 in pixel
	global x3_px , y3_px
	x3_px = data.data[0]
	y3_px = data.data[1]
def callback_rob3_CurrentPose(data):# for pose of the robot 3 in cm
	global x3,y3
	x3 = data.data[0]
	y3 = data.data[1]
####################################################
def callback_rob4_pose_px(data):# for pose of the robot 4 in pixel
	global x4_px , y4_px
	x4_px = data.data[0]
	y4_px = data.data[1]
def callback_rob4_CurrentPose(data):# for pose of the robot 4 in cm
	global x4,y4
	x4 = data.data[0]
	y4 = data.data[1]
####################################################
def callback_Ob1_pose_cm(data):# for pose of the obstacle 1 in cm
	global x_o1_cm , y_o1_cm
	x_o1_cm = data.data[0]
	y_o1_cm = data.data[1]
def callback_Ob1_pose_px(data):# for pose of the obstacle 1 in pixel
	global x_o1_px , y_o1_px
	x_o1_px = data.data[0]
	y_o1_px = data.data[1]
####################################################
def callback_Ob2_pose_cm(data):# for pose of the obstacle 2 in cm
	global x_o2_cm , y_o2_cm
	x_o2_cm = data.data[0]
	y_o2_cm = data.data[1]
def callback_Ob2_pose_px(data):# for pose of the obstacle 2 in pixel
	global x_o2_px , y_o2_px
	x_o2_px = data.data[0]
	y_o2_px = data.data[1]
####################################################
def callback_formation_possible_flag(data):# if formation is possible or not
	global possible_flag
	possible_flag = data.data
####################################################
def callback_set_new_leader(data):
	global leaderID
	leaderID= data.data
	best_leader_xy ()
####################################################
def callback_shape_length(data):
	global shapeLength
	shapeLength= data.data

####################################################
def callback_formation_shape_name(data):
	global shapeName
	shapeName= data.data

def check_quad ():
	global quad, trail
	global leaderID
	global leader_x
	global leader_y
	global leader_final_goal
	global square_corner1_x, square_corner1_y, square_corner2_x, square_corner2_y, square_corner3_x, square_corner3_y, square_corner4_x, square_corner4_y

	global calculated
	if (trail < 16) :
		if leaderID == 0 :
			leader_x = x1_px
			leader_y = y1_px
		elif leaderID == 1:
			leader_x = x2_px
			leader_y = y2_px
		elif leaderID == 2:
			leader_x = x3_px
			leader_y = y3_px
		elif leaderID == 3:
			leader_x = x4_px
			leader_y = y4_px

		#get the lengths between leader and other robots s1,s2,s3,s4
		s1 = ( (leader_x - x1_px) ** 2  + (leader_y - y1_px) ** 2 ) ** 0.5
		#if s1 == 0 :
			#s1 = s1 + 100

		s2 = ( (leader_x - x2_px) ** 2  + (leader_y - y2_px) ** 2 ) ** 0.5
		#if s2 == 0 :
			#s2 = s2 + 100

		s3 = ( (leader_x - x3_px) ** 2  + (leader_y - y3_px) ** 2 ) ** 0.5
		#if s3 == 0 :
			#s3 = s3 + 100

		s4 = ( (leader_x - x4_px) ** 2  + (leader_y - y4_px) ** 2 ) ** 0.5
		#if s4 == 0 :
			#s4 = s4 + 100
		if s1 == 0 :
			s1 = s1 + 100
		elif s2 == 0 :
			s2 = s2 + 100
		elif s3 == 0 :
			s3 = s3 + 100
		elif s4 == 0 :
			s4 = s4 + 100
		#get the shortest length between leader and follower 1
		l1 = min (s1,s2,s3,s4)

		#get follower 1 ID and x,y
		if l1 == s1 :
			follower1ID = 0
			follower1_x = x1_px
			follower1_y = y1_px
		elif l1 == s2 :
			follower1ID = 1
			follower1_x = x2_px
			follower1_y = y2_px
		elif l1 == s3 :
			follower1ID = 2
			follower1_x = x3_px
			follower1_y = y3_px
		elif l1 == s4 :
			follower1ID = 3
			follower1_x = x4_px
			follower1_y = y4_px

	################################################################################

		#get the lengths between follower 1 and other robots s1,s2,s3,s4
		s1 = ( (follower1_x - x1_px) ** 2  + (follower1_y - y1_px) ** 2 ) ** 0.5
		#if s1 == 0 or s1 == l1 :
			#s1 = s1 + 100

		s2 = ( (follower1_x - x2_px) ** 2  + (follower1_y - y2_px) ** 2 ) ** 0.5
		#if s2 == 0 or s2 == l1 :
			#s2 = s2 + 100

		s3 = ( (follower1_x - x3_px) ** 2  + (follower1_y - y3_px) ** 2 ) ** 0.5
		#if s3 == 0 or s3 == l1 :
			#s3 = s3 + 100

		s4 = ( (follower1_x - x4_px) ** 2  + (follower1_y - y4_px) ** 2 ) ** 0.5
		#if s4 == 0 or s4 == l1 :
			#s4 = s4 + 100

		if s1 == 0 :
			s1 = s1 + 100
		elif s2 == 0 :
			s2 = s2 + 100
		elif s3 == 0 :
			s3 = s3 + 100
		elif s4 == 0 :
			s4 = s4 + 100

		if s1 == l1 :
			s1 = s1 + 100
		elif s2 == l1 :
			s2 = s2 + 100
		elif s3 == l1 :
			s3 = s3 + 100
		elif s4 == l1 :
			s4 = s4 + 100


		#get the shortest length between follower 1 and follower 2
		l2 = min (s1,s2,s3,s4)

		#get follower 2 ID and x,y
		if l2 == s1 :
			follower2ID = 0
			follower2_x = x1_px
			follower2_y = y1_px
		elif l2 == s2 :
			follower2ID = 1
			follower2_x = x2_px
			follower2_y = y2_px
		elif l2 == s3 :
			follower2ID = 2
			follower2_x = x3_px
			follower2_y = y3_px
		elif l2 == s4 :
			follower2ID = 3
			follower2_x = x4_px
			follower2_y = y4_px

	################################################################################

		#get the lengths between follower 2 and other robots s1,s2,s3,s4
		s1 = ( (follower2_x - x1_px) ** 2  + (follower2_y - y1_px) ** 2 ) ** 0.5
		#if s1 == 0 or s1 == l2 :
			#s1 = s1 + 100
		s2 = ( (follower2_x - x2_px) ** 2  + (follower2_y - y2_px) ** 2 ) ** 0.5
		#if s2 == 0 or s2 == l2 :
			#s2 = s2 + 100

		s3 = ( (follower2_x - x3_px) ** 2  + (follower2_y - y3_px) ** 2 ) ** 0.5
		#if s3 == 0 or s3 == l2 :
			#s3 = s3 + 100

		s4 = ( (follower2_x - x4_px) ** 2  + (follower2_y - y4_px) ** 2 ) ** 0.5
		#if s4 == 0 or s4 == l2 :
			#s4 = s4 + 100
		if s1 == 0 :
			s1 = s1 + 100
		elif s2 == 0 :
			s2 = s2 + 100
		elif s3 == 0 :
			s3 = s3 + 100
		elif s4 == 0 :
			s4 = s4 + 100

		if s1 == l2 :
			s1 = s1 + 100
		elif s2 == l2 :
			s2 = s2 + 100
		elif s3 == l2 :
			s3 = s3 + 100
		elif s4 == l2 :
			s4 = s4 + 100

		#get the shortest length between follower 2 and follower 3
		l3 = min (s1,s2,s3,s4)

		#get follower 3 ID and x,y
		if l3 == s1 :
			follower3ID = 0
			follower3_x = x1_px
			follower3_y = y1_px
		elif l3 == s2 :
			follower3ID = 1
			follower3_x = x2_px
			follower3_y = y2_px
		elif l3 == s3 :
			follower3ID = 2
			follower3_x = x3_px
			follower3_y = y3_px
		elif l3 == s4 :
			follower3ID = 3
			follower3_x = x4_px
			follower3_y = y4_px

		############################################################################
		#check quads for leader
		if (shapeName == 'square'):
			if ( quad == 1 ) :
				#quad 1
				square_corner1_x = leader_x
				square_corner1_y = leader_y

				square_corner2_x = leader_x + (shapeLength/25)
				square_corner2_y = leader_y

				square_corner3_x = leader_x
				square_corner3_y = leader_y + (shapeLength/25)

				square_corner4_x = leader_x + (shapeLength/25)
				square_corner4_y = leader_y + (shapeLength/25)

			elif ( quad == 2 ) :
				#quad 2
				square_corner1_x = leader_x
				square_corner1_y = leader_y

				square_corner2_x = leader_x - (shapeLength/25)
				square_corner2_y = leader_y

				square_corner3_x = leader_x
				square_corner3_y = leader_y + (shapeLength/25)

				square_corner4_x = leader_x - (shapeLength/25)
				square_corner4_y = leader_y + (shapeLength/25)

			elif ( quad == 3 ) :
				#quad 3
				square_corner1_x = leader_x
				square_corner1_y = leader_y

				square_corner2_x = leader_x - (shapeLength/25)
				square_corner2_y = leader_y

				square_corner3_x = leader_x
				square_corner3_y = leader_y - (shapeLength/25)

				square_corner4_x = leader_x - (shapeLength/25)
				square_corner4_y = leader_y - (shapeLength/25)

			if ( quad == 4 ) :
				#quad 4
				square_corner1_x = leader_x
				square_corner1_y = leader_y

				square_corner2_x = leader_x + (shapeLength/25)
				square_corner2_y = leader_y

				square_corner3_x = leader_x
				square_corner3_y = leader_y - (shapeLength/25)

				square_corner4_x = leader_x + (shapeLength/25)
				square_corner4_y = leader_y - (shapeLength/25)

		#check OBSTACLES
		if (((square_corner1_x == x_o1_px) and (square_corner1_y == y_o1_px)) or ((square_corner1_x == x_o2_px) and (square_corner1_y == y_o2_px)) or ((square_corner2_x == x_o1_px) and (square_corner2_y == y_o1_px)) or ((square_corner2_x == x_o2_px) and (square_corner2_y == y_o2_px)) or ((square_corner3_x == x_o1_px) and (square_corner3_y == y_o1_px)) or ((square_corner3_x == x_o2_px) and (square_corner3_y == y_o2_px)) or ((square_corner4_x == x_o1_px) and (square_corner4_y == y_o1_px)) or ((square_corner4_x == x_o2_px) and (square_corner4_y == y_o2_px)) or (square_corner1_x < 0 or square_corner1_x > 6 ) or  (square_corner1_y < 0 or square_corner1_y > 6 ) or (square_corner2_x < 0 or square_corner2_x > 6 ) or  (square_corner2_y < 0 or square_corner2_y > 6 ) or	(square_corner3_x < 0 or square_corner3_x > 6 ) or  (square_corner3_y < 0 or square_corner3_y > 6 ) or (square_corner4_x < 0 or square_corner4_x > 6 ) or  (square_corner4_y < 0 or square_corner4_y > 6 )) :
			quad = quad + 1
			if (quad == 5):
				quad = 1
				leaderID = leaderID + 1
				if (leaderID == 4):
					leaderID = 0
				trail = trail + 1

			check_quad ()

		else :
			best_leader_ID = leaderID
			best_quad = quad
			print ()
			print ('leaderID' , leaderID)
			print ('quad' , best_quad)
			print ('x1_px and y1_px', x1_px , y1_px )
			print ('x2_px and y2_px', x2_px , y2_px)
			print ('x3_px and y3_px', x3_px , y3_px)
			print ('x4_px and y4_px', x4_px , y4_px)
			print ('Leader x and y ', leader_x , leader_y )
			pub_quad.publish(quad)
			pub_leader_id.publish(leaderID+1)
			pub_leader_final_goal.publish(Int32MultiArray(data=[leader_x, leader_y]))
			calculated = 1

	else :
		best_leader_xy ()

def listener():
	# Subscribe to robot1 position from the over head cam
	rospy.Subscriber('robot1' ,Int32MultiArray,callback_rob1_pose_px)
	rospy.Subscriber('rob1_CurrentPose' ,Int32MultiArray,callback_rob1_CurrentPose)
	# Subscribe to robot2 position from the over head cam
	rospy.Subscriber('robot2' ,Int32MultiArray,callback_rob2_pose_px)
	rospy.Subscriber('rob2_CurrentPose' ,Int32MultiArray,callback_rob2_CurrentPose)
	# Subscribe to robot3 position from the over head cam
	rospy.Subscriber('robot3' ,Int32MultiArray,callback_rob3_pose_px)
	rospy.Subscriber('rob3_CurrentPose' ,Int32MultiArray,callback_rob3_CurrentPose)
	# Subscribe to robot4 position from the over head cam
	rospy.Subscriber('robot4' ,Int32MultiArray,callback_rob4_pose_px)
	rospy.Subscriber('rob4_CurrentPose' ,Int32MultiArray,callback_rob4_CurrentPose)

	# Subscribe to obstcales position from the over head cam
	rospy.Subscriber('obst1', Int32MultiArray, callback_Ob1_pose_px)
	rospy.Subscriber('obst1_CurrentPose', Int32MultiArray, callback_Ob1_pose_cm)

	rospy.Subscriber('obst2', Int32MultiArray, callback_Ob2_pose_px)
	rospy.Subscriber('obst2_CurrentPose', Int32MultiArray, callback_Ob2_pose_cm)

	#if formation is possible or not
	#rospy.Subscriber('formation_possible_flag', Byte, callback_formation_possible_flag)

	#which robot is the leader
	rospy.Subscriber('potential_leader', Int32, callback_set_new_leader)

	#shape length
	rospy.Subscriber('shape_length', Int32, callback_shape_length)

	#shape name #square line diagonal column
	rospy.Subscriber('formation_shape_name', String, callback_formation_shape_name)

	#publish leader x,y

def takeClosest(myList, myNumber):
	#"""
	#Assumes myList is sorted. Returns closest value to myNumber.

	#If two numbers are equally close, return the smallest number.
	#"""
	pos = bisect_left(myList, myNumber)
	if pos == 0:
		return myList[0]
	if pos == len(myList):
		return myList[-1]
	before = myList[pos - 1]
	after = myList[pos]
	if after - myNumber < myNumber - before:
	   return after
	else:
	   return before

def square (minx, miny, intx, inty, leader_x, leader_y , min_x, min_y) :
	global best_leader_x , best_leader_y
	global square_corner1_x , square_corner1_y
	global square_corner2_x , square_corner2_y
	global square_corner3_x , square_corner3_y
	global square_corner4_x , square_corner4_y
	global r

	repeat = 7 - (shapeLength/25)
	#mp_y = [1,2,3,4,5,6,7,8,9]
	for x in range (0, repeat):
		y = 1 + (shapeLength/25)/2.0 + x
		mp_x.append (y) #list of midpoints for square shape in x axis
		mp_y.append (y) #list of midpoints for square shape in y axis
		min_x.append (y - intx)
		min_y.append (y - inty)
	minx = takeClosest(min_x, r)
	miny = takeClosest(min_y, r)

	square_mp_x = intx + minx #desired shape midpoints
	square_mp_y = inty + miny
	square_corner1_x = square_mp_x + (shapeLength/25)/2.0 #square corners x,y
	square_corner1_y = square_mp_y + (shapeLength/25)/2.0
	square_corner2_x = square_mp_x + (shapeLength/25)/2.0
	square_corner2_y = square_mp_y - (shapeLength/25)/2.0
	square_corner3_x = square_mp_x - (shapeLength/25)/2.0
	square_corner3_y = square_mp_y + (shapeLength/25)/2.0
	square_corner4_x = square_mp_x - (shapeLength/25)/2.0
	square_corner4_y = square_mp_y - (shapeLength/25)/2.0
	#check obstacles
	if (((square_corner1_x == x_o1_px) and (square_corner1_y == y_o1_px)) or
		((square_corner1_x == x_o2_px) and (square_corner1_y == y_o2_px)) or
		((square_corner2_x == x_o1_px) and (square_corner2_y == y_o1_px)) or
		((square_corner2_x == x_o2_px) and (square_corner2_y == y_o2_px)) or
		((square_corner3_x == x_o1_px) and (square_corner3_y == y_o1_px)) or
		((square_corner3_x == x_o2_px) and (square_corner3_y == y_o2_px)) or
		((square_corner4_x == x_o1_px) and (square_corner4_y == y_o1_px)) or
		((square_corner4_x == x_o2_px) and (square_corner4_y == y_o2_px))) :
			r = r + 1
			square (minx, miny, intx, inty, leader_x, leader_y , min_x, min_y)

	else :
		#distance between leader and square corners
		s1 = ( (leader_x - square_corner1_x) ** 2  + (leader_y - square_corner1_y) ** 2 ) ** 0.5
		s2 = ( (leader_x - square_corner2_x) ** 2  + (leader_y - square_corner2_y) ** 2 ) ** 0.5
		s3 = ( (leader_x - square_corner3_x) ** 2  + (leader_y - square_corner3_y) ** 2 ) ** 0.5
		s4 = ( (leader_x - square_corner4_x) ** 2  + (leader_y - square_corner4_y) ** 2 ) ** 0.5
		#get the closest corner
		l1 = min (s1,s2,s3,s4)
		if l1 == s1 :
			closest_corner = 1
			best_leader_x = square_corner1_x
			best_leader_y = square_corner1_y
		elif l1 == s2 :
			closest_corner = 2
			best_leader_x = square_corner2_x
			best_leader_y = square_corner2_y
		elif l1 == s3 :
			closest_corner = 3
			best_leader_x = square_corner3_x
			best_leader_y = square_corner3_y
		elif l1 == s4 :
			closest_corner = 4
			best_leader_x = square_corner4_x
			best_leader_y = square_corner4_y
		#print best leader x,y
		print ('best leader x,y' , best_leader_x , best_leader_y )
		print ('corner 1 x,y' , square_corner1_x , square_corner1_y)
		print ('corner 2 x,y' , square_corner2_x , square_corner2_y)
		print ('corner 3 x,y' , square_corner3_x , square_corner3_y)
		print ('corner 4 x,y' , square_corner4_x , square_corner4_y)


def line (minx, miny, intx, inty, leader_x, leader_y , min_x, min_y) :
	global best_leader_x , best_leader_y
	global line_corner1_x , square_corner1_y
	global line_corner2_x , square_corner2_y
	global line_corner3_x , square_corner3_y
	global line_corner4_x , square_corner4_y,r

	repeat = 10 - 3*(shapeLength/25)

	mp_y = [1,2,3,4,5,6,7,8,9]

	for x in range (0, 9):
		min_y.append(x - inty)

	for x in range (0, repeat):
		y = 1.5*(shapeLength/25) + x
		mp_x.append (y) #list of midpoints for line shape in x axis
		min_x.append (y - intx)

	minx = takeClosest(min_x, r)
	miny = takeClosest(min_y, r)


	line_mp_x = intx + minx #desired shape midpoints
	line_mp_y = inty + miny


	line_corner1_x = line_mp_x + (shapeLength/25)/2.0 #square corners x,y
	line_corner1_y = line_mp_y
	line_corner2_x = line_mp_x + 3*(shapeLength/25)/2.0
	line_corner2_y = line_mp_y 
	line_corner3_x = line_mp_x - (shapeLength/25)/2.0
	line_corner3_y = line_mp_y
	line_corner4_x = line_mp_x - 3*(shapeLength/25)/2.0
	line_corner4_y = line_mp_y

	#check obstacles
	if (((line_corner1_x == x_o1_px) and (line_corner1_y == y_o1_px)) or
		((line_corner1_x == x_o2_px) and (line_corner1_y == y_o2_px)) or
		((line_corner2_x == x_o1_px) and (line_corner2_y == y_o1_px)) or
		((line_corner2_x == x_o2_px) and (line_corner2_y == y_o2_px)) or
		((line_corner3_x == x_o1_px) and (line_corner3_y == y_o1_px)) or
		((line_corner3_x == x_o2_px) and (line_corner3_y == y_o2_px)) or
		((line_corner4_x == x_o1_px) and (line_corner4_y == y_o1_px)) or
		((line_corner4_x == x_o2_px) and (line_corner4_y == y_o2_px))) :
			r = r + 1
			line (minx, miny, intx, inty, leader_x, leader_y , min_x, min_y)

	else :
		#distance between leader and square corners
		s1 = ( (leader_x - line_corner1_x) ** 2  + (leader_y - line_corner1_y) ** 2 ) ** 0.5
		s2 = ( (leader_x - line_corner2_x) ** 2  + (leader_y - line_corner2_y) ** 2 ) ** 0.5
		s3 = ( (leader_x - line_corner3_x) ** 2  + (leader_y - line_corner3_y) ** 2 ) ** 0.5
		s4 = ( (leader_x - line_corner4_x) ** 2  + (leader_y - line_corner4_y) ** 2 ) ** 0.5
		#get the closest corner
		l1 = min (s1,s2,s3,s4)
		if l1 == s1 :
			closest_corner = 1
			best_leader_x = line_corner1_x
			best_leader_y = line_corner1_y
		elif l1 == s2 :
			closest_corner = 2
			best_leader_x = line_corner2_x
			best_leader_y = line_corner2_y
		elif l1 == s3 :
			closest_corner = 3
			best_leader_x = line_corner3_x
			best_leader_y = line_corner3_y
		elif l1 == s4 :
			closest_corner = 4
			best_leader_x = line_corner4_x
			best_leader_y = line_corner4_y
		#print best leader x,y
		print ('best leader x,y' , best_leader_x , best_leader_y )
[0, 0]
[0, 0]
[0, 0]
[0, 0]
[0, 0]
[0, 0]
def best_leader_xy ():
	global quad
	global leaderID
	global leader_x
	global leader_y
	global leader_final_goal,trail

	if leaderID == 0 :
		leader_x = x1_px
		leader_y = y1_px
	elif leaderID == 1:
		leader_x = x2_px
		leader_y = y2_px
	elif leaderID == 2:
		leader_x = x3_px
		leader_y = y3_px
	elif leaderID == 3:
		leader_x = x4_px
		leader_y = y4_px

	#get the lengths between leader and other robots s1,s2,s3,s4
	s1 = ( (leader_x - x1_px) ** 2  + (leader_y - y1_px) ** 2 ) ** 0.5
	#if s1 == 0 :
		#s1 = s1 + 100

	s2 = ( (leader_x - x2_px) ** 2  + (leader_y - y2_px) ** 2 ) ** 0.5
	#if s2 == 0 :
		#s2 = s2 + 100

	s3 = ( (leader_x - x3_px) ** 2  + (leader_y - y3_px) ** 2 ) ** 0.5
	#if s3 == 0 :
		#s3 = s3 + 100

	s4 = ( (leader_x - x4_px) ** 2  + (leader_y - y4_px) ** 2 ) ** 0.5
	#if s4 == 0 :
		#s4 = s4 + 100
	if s1 == 0 :
		s1 = s1 + 100
	elif s2 == 0 :
		s2 = s2 + 100
	elif s3 == 0 :
		s3 = s3 + 100
	elif s4 == 0 :
		s4 = s4 + 100
	#get the shortest length between leader and follower 1
	l1 = min (s1,s2,s3,s4)

	#get follower 1 ID and x,y
	if l1 == s1 :
		follower1ID = 0
		follower1_x = x1_px
		follower1_y = y1_px
	elif l1 == s2 :
		follower1ID = 1
		follower1_x = x2_px
		follower1_y = y2_px
	elif l1 == s3 :
		follower1ID = 2
		follower1_x = x3_px
		follower1_y = y3_px
	elif l1 == s4 :
		follower1ID = 3
		follower1_x = x4_px
		follower1_y = y4_px

################################################################################

	#get the lengths between follower 1 and other robots s1,s2,s3,s4
	s1 = ( (follower1_x - x1_px) ** 2  + (follower1_y - y1_px) ** 2 ) ** 0.5
	#if s1 == 0 or s1 == l1 :
		#s1 = s1 + 100

	s2 = ( (follower1_x - x2_px) ** 2  + (follower1_y - y2_px) ** 2 ) ** 0.5
	#if s2 == 0 or s2 == l1 :
		#s2 = s2 + 100

	s3 = ( (follower1_x - x3_px) ** 2  + (follower1_y - y3_px) ** 2 ) ** 0.5
	#if s3 == 0 or s3 == l1 :
		#s3 = s3 + 100

	s4 = ( (follower1_x - x4_px) ** 2  + (follower1_y - y4_px) ** 2 ) ** 0.5
	#if s4 == 0 or s4 == l1 :
		#s4 = s4 + 100

	if s1 == 0 :
		s1 = s1 + 100
	elif s2 == 0 :
		s2 = s2 + 100
	elif s3 == 0 :
		s3 = s3 + 100
	elif s4 == 0 :
		s4 = s4 + 100

	if s1 == l1 :
		s1 = s1 + 100
	elif s2 == l1 :
		s2 = s2 + 100
	elif s3 == l1 :
		s3 = s3 + 100
	elif s4 == l1 :
		s4 = s4 + 100


	#get the shortest length between follower 1 and follower 2
	l2 = min (s1,s2,s3,s4)

	#get follower 2 ID and x,y
	if l2 == s1 :
		follower2ID = 0
		follower2_x = x1_px
		follower2_y = y1_px
	elif l2 == s2 :
		follower2ID = 1
		follower2_x = x2_px
		follower2_y = y2_px
	elif l2 == s3 :
		follower2ID = 2
		follower2_x = x3_px
		follower2_y = y3_px
	elif l2 == s4 :
		follower2ID = 3
		follower2_x = x4_px
		follower2_y = y4_px

################################################################################

	#get the lengths between follower 2 and other robots s1,s2,s3,s4
	s1 = ( (follower2_x - x1_px) ** 2  + (follower2_y - y1_px) ** 2 ) ** 0.5
	#if s1 == 0 or s1 == l2 :
		#s1 = s1 + 100
	s2 = ( (follower2_x - x2_px) ** 2  + (follower2_y - y2_px) ** 2 ) ** 0.5
	#if s2 == 0 or s2 == l2 :
		#s2 = s2 + 100

	s3 = ( (follower2_x - x3_px) ** 2  + (follower2_y - y3_px) ** 2 ) ** 0.5
	#if s3 == 0 or s3 == l2 :
		#s3 = s3 + 100

	s4 = ( (follower2_x - x4_px) ** 2  + (follower2_y - y4_px) ** 2 ) ** 0.5
	#if s4 == 0 or s4 == l2 :
		#s4 = s4 + 100
	if s1 == 0 :
		s1 = s1 + 100
	elif s2 == 0 :
		s2 = s2 + 100
	elif s3 == 0 :
		s3 = s3 + 100
	elif s4 == 0 :
		s4 = s4 + 100

	if s1 == l2 :
		s1 = s1 + 100
	elif s2 == l2 :
		s2 = s2 + 100
	elif s3 == l2 :
		s3 = s3 + 100
	elif s4 == l2 :
		s4 = s4 + 100

	#get the shortest length between follower 2 and follower 3
	l3 = min (s1,s2,s3,s4)

	#get follower 3 ID and x,y
	if l3 == s1 :
		follower3ID = 0
		follower3_x = x1_px
		follower3_y = y1_px
	elif l3 == s2 :
		follower3ID = 1
		follower3_x = x2_px
		follower3_y = y2_px
	elif l3 == s3 :
		follower3ID = 2
		follower3_x = x3_px
		follower3_y = y3_px
	elif l3 == s4 :
		follower3ID = 3
		follower3_x = x4_px
		follower3_y = y4_px

################################################################################
	#get l4
	l4 = ( (follower3_x - leader_x) ** 2  + (follower3_y - leader_y) ** 2 ) ** 0.5

	#A = (leader_x, leader_y)
	#B = (follower2_x, follower2_y)
	#C = (follower1_x, follower1_y)
	#D = (follower3_x, follower3_y)

	#line1 = LineString([A, B])
	#line2 = LineString([C, D])


	#int_pt = line1.intersection(line2

	intx = (leader_x + follower1_x + follower2_x + follower3_x) / 4.0
	inty = (leader_y + follower1_y + follower2_y + follower3_y) / 4.0
	point_of_intersection = intx, inty

	#repeat = 7 - shapeLength

	#for x in range (0, repeat):
	#    y = 1 + shapeLength/2.0 + x
	#    mp_x.append (y) #list of midpoints for square shape in x axis
	#    mp_y.append (y) #list of midpoints for square shape in y axis
	#    min_x.append (y - intx)
	#    min_y.append (y - inty)
	#minx = takeClosest(min_x, 0)
	#miny = takeClosest(min_y, 0)
	if (shapeName == 'square'):
		square (minx, miny, intx, inty, leader_x, leader_y, min_x, min_y)

	elif (shapeName == 'line'):
		line (minx, miny, intx, inty, leader_x, leader_y, min_x, min_y)


	pub_quad.publish(quad)
	pub_leader_id.publish(leaderID)
	pub_leader_final_goal.publish(Int32MultiArray(data=leader_final_goal))

	#print(point_of_intersection
	#print(intx , inty )
	#print (l1 , l2 , l3 , l4)
	print ('Leader is r' , leaderID , 'x,y' , leader_x , leader_y)
	print ('follower 1 is r' , follower1ID , 'x,y' , follower1_x , follower1_y)
	print ('follower 2 is r' , follower2ID , 'x,y' , follower2_x , follower2_y)
	print ('follower 3 is r' , follower3ID , 'x,y' , follower3_x , follower3_y)
	print ('x1_px and y1_px', x1_px , y1_px )
	print ('x2_px and y2_px', x2_px , y2_px)
	print ('x3_px and y3_px', x3_px , y3_px)
	print ('x4_px and y4_px', x4_px , y4_px)
	#print (minx)
	#print (miny)


if __name__ == '__main__':
	rospy.init_node('best_leader_start_point')
	try:
		listener()
		time.sleep(3)
		check_quad ()
		
		while not rospy.is_shutdown():
			rospy.spin()
			
	except rospy.ROSInterruptException:
		pass
