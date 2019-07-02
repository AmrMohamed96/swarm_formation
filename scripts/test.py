#!/usr/bin/env python
from __future__ import division
import rospy
import math
import time
import numpy as np
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray, Int32, Byte

def test():
    rospy.init_node('formation_tester')
    pub1 = rospy.Publisher('align_axis_rob1', Int32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('align_axis_rob2', Int32MultiArray, queue_size=10)
    pub3 = rospy.Publisher('align_axis_rob3', Int32MultiArray, queue_size=10)
    pub4 = rospy.Publisher('align_axis_rob4', Int32MultiArray, queue_size=10)
    pub5 = rospy.Publisher('shape_corner_rob1', Int32MultiArray, queue_size=10)
    pub6 = rospy.Publisher('shape_corner_rob2', Int32MultiArray, queue_size=10)
    pub7 = rospy.Publisher('shape_corner_rob3', Int32MultiArray, queue_size=10)
    pub8 = rospy.Publisher('shape_corner_rob4', Int32MultiArray, queue_size=10)

    test = Int32MultiArray()
    test.data = [1,1]

    pos =  Int32MultiArray()
    pos.data = [5,5]

    newLead = rospy.Publisher('set_new_leader', Byte, queue_size=1)
    rospy.loginfo("Sleeping for 5 seconds")
    time.sleep(5)
    newLead.publish(1)
    rospy.loginfo("Published new leader once")

    resetLead = rospy.Publisher('reset_leader_stats', Byte, queue_size=1)
    rospy.loginfo("Sleeping for 5 seconds")
    time.sleep(5)
    resetLead.publish(1)
    rospy.loginfo("Reset Leader Status")

    Rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub1.publish(test)
        pub2.publish(test)
        pub3.publish(test)
        pub4.publish(test)

        pub5.publish(pos)
        pub6.publish(pos)
        pub7.publish(pos)
        pub8.publish(pos)

        Rate.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
    finally:
        print "Exiting Node"
