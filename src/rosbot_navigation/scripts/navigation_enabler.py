#!/usr/bin/env python

# Read the navigation command (from /cmd_vel_raw topic) and forward it to the /cmd_vel topic,
# if the navigation is enabled (from /navigation_enabler topic).

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


enable_navigation = False

def navigation_callback(data):
    global enable_navigation
    enable_navigation = data.data

def cmd_vel_callback(data):
    global enable_navigation
    if enable_navigation:
        cmd_vel_pub.publish(data)
    else:
        cmd_vel_pub.publish(Twist())

rospy.init_node('navigation_enabler')
rospy.Subscriber('navigation_enabler', Bool, navigation_callback)
rospy.Subscriber('cmd_vel_raw', Twist, cmd_vel_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.spin()
