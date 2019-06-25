#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time

# initialize variables
shapes=''
c2c_distance_px='0'
robots_num = '0'

# initialize ros node
rospy.init_node('shape_selector')

# set publishers
pub= rospy.Publisher('req_shape',String, queue_size=10)
#C2C_distance is the distance from corner to corner
pub1= rospy.Publisher('c2c_distance_px',String, queue_size=10)
pub2 = rospy.Publisher('robots_num', String, queue_size=10)


def shape():
# ask for input of shape and read it
	global shapes, c2c_distance_px, robots_num
	shapes = raw_input("Choose a formation from (line, column, triangle, diagonal, l-shape, square): ")
	c2c_distance_px = raw_input("Choose the corner to corner distance: ")
	robots_num = raw_input("Enter number of robots in the system: ")

if __name__== '__main__':
	shape()
	time.sleep(5)
	pub.publish(shapes)
	pub1.publish(c2c_distance_px)
	pub2.publish(robots_num)
	rospy.loginfo('Sent Shape as: {}'.format(shapes))
	rospy.loginfo('Sent C2C Distance as: {}'.format(c2c_distance_px))
	rospy.loginfo('Sent Number of Robots as: {}'.format(robots_num))
