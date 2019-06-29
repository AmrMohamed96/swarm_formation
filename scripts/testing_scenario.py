#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32MultiArray
import numpy as np
from math import pi
from time import sleep

# Robots Positions
# robot 1 is at pixels (2,2) looking at the +ve X-axis direction
rob1_pos = [35,35, (-pi/2)*100]

# robot 2 is at pixels (4,2) looking at the +ve X-axis direction
rob2_pos = [70,35, (-pi/2)*100]

# robot 3 is at pixels (3,4) looking at the +ve X-axis direction
rob3_pos = [52, 70, (-pi/2)*100]

# robot 4 is at pixels (7,8) lookng at the +ve X-Axis direction
rob4_pos = [122,140, (-pi/2)*100]

# Collecting all robot positions in one list
all_robots_pos = np.reshape([rob1_pos[0],rob1_pos[1], rob2_pos[0],rob2_pos[1],rob3_pos[0],rob3_pos[1], rob4_pos[0],rob4_pos[1]], (8))


def test_scenario_talker():
    # initializing the node
    rospy.init_node('testing_scenario_node')
    rospy.loginfo('%s started.' % rospy.get_name())

    # position publishers
    rob1 = rospy.Publisher('rob1_CurrentPose', Int32MultiArray, queue_size=10)
    rob2 = rospy.Publisher('rob2_CurrentPose', Int32MultiArray, queue_size=10)
    rob3 = rospy.Publisher('rob3_CurrentPose', Int32MultiArray, queue_size=10)
    rob4 = rospy.Publisher('rob4_CurrentPose', Int32MultiArray, queue_size=10)
    all = rospy.Publisher('robots_current_poses', Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        # publishing robot positions
        rob1.publish( Int32MultiArray(data=rob1_pos) )
        rob2.publish( Int32MultiArray(data=rob2_pos) )
        rob3.publish( Int32MultiArray(data=rob3_pos) )
        rob4.publish( Int32MultiArray(data=rob4_pos) )
        all.publish( Int32MultiArray(data=all_robots_pos) )

        #adding 0.5 second delay to simulate camera delay
        #sleep(0.5)

if __name__ == '__main__':
    try:
        test_scenario_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo('%s is closed.' % rospy.get_name())
