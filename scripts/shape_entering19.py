#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time

# initialize variables
shapes=''
c2c_distance_px='0'
robots_num = '0'

system_latency = 5

# initialize ros node
rospy.init_node('shape_selector')
rospy.loginfo('%s started' % rospy.get_name())

# shape setting node publishers
pub= rospy.Publisher('req_shape',String, queue_size=10)
pub1= rospy.Publisher('c2c_distance_px',String, queue_size=10)
pub2 = rospy.Publisher('robots_num', String, queue_size=10)

rospy.loginfo('Sleeping for %d to ensure connections to publishers.' % system_latency)
time.sleep(system_latency)


def shape():
	# ask for input of shape and read it
	global shapes, c2c_distance_px, robots_num

	shapes = raw_input("Choose a formation from (line, column, triangle, diagonal, l-shape, square): ")
	c2c_distance_px = raw_input("Choose the corner to corner distance: ")
	robots_num = raw_input("Enter number of robots in the system: ")

if __name__== '__main__':
	shape()

	pub.publish(shapes)
	pub1.publish(c2c_distance_px)
	pub2.publish(robots_num)
	
	rospy.loginfo('Sent Shape as: {}'.format(shapes))
	rospy.loginfo('Sent C2C Distance as: {}'.format(c2c_distance_px))
	rospy.loginfo('Sent Number of Robots as: {}'.format(robots_num))
