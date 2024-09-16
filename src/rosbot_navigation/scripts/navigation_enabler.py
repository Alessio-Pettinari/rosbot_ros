#!/usr/bin/env python3

# Read the navigation command (from /cmd_vel_raw topic) and forward it to the /cmd_vel topic,
# if the navigation is enabled (from /navigation_enabler topic).

import rospy
from geometry_msgs.msg import Twist, Pose, PoseArray
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import numpy as np

path_ugv = PoseArray()
enable_navigation = False

def navigation_callback(data):
    global enable_navigation
    enable_navigation = data.data
    rospy.loginfo(f"flag: {enable_navigation}")


## Callback UGV goal reached
def UGV_goal_reach_callback(data):
    global flag_UGVgoalReach
    flag_UGVgoalReach = data.data
    rospy.loginfo(f"flagUGVGOAL: {flag_UGVgoalReach}")


def cmd_vel_callback(data):
    global enable_navigation, flag_UGVgoalReach
    if enable_navigation : 
        cmd_vel_pub.publish(data)
    elif flag_UGVgoalReach:
        cmd_vel_pub.publish(Twist())
    else:
        cmd_vel_pub.publish(Twist())



def UGV_global_planner_callback(data):
    global path_ugv
    path_ugv = PoseArray()
    path_ugv.header = data.header
        
    for pose in data.poses:
        pose_msg = Pose()
        pose_msg.position.x = pose.pose.position.x
        pose_msg.position.y = pose.pose.position.y
        pose_msg.position.z = pose.pose.position.z

        path_ugv.poses.append(pose_msg)
    
    ugv_path_pub.publish(path_ugv)
   

rospy.init_node('navigation_enabler')
rospy.Subscriber('/first/navigation_enabler', Bool, navigation_callback)
rospy.Subscriber('/first/cmd_vel_raw', Twist, cmd_vel_callback)
rospy.Subscriber('/first/first_move_base/NavfnROS/plan', Path, UGV_global_planner_callback)
rospy.Subscriber('/first/GoalReached', Bool, UGV_goal_reach_callback)
cmd_vel_pub = rospy.Publisher('/first/cmd_vel', Twist, queue_size=10)
ugv_path_pub = rospy.Publisher('/first/global_path', PoseArray, queue_size=10)

rate = rospy.Rate(10) 
while not rospy.is_shutdown():
    if path_ugv.poses: 
        ugv_path_pub.publish(path_ugv)
    rate.sleep()

