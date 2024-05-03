#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
import math
import numpy as np
from scipy.spatial import KDTree

base_waypoints = None
waypoints_2d = None
waypoint_tree = None
pose = None
LOOKAHEAD_WPS = 10 # Number of waypoints we will publish. You can change this number
def pose_cb(msg):
        global pose
        pose = msg
def waypoints_cb(waypoints):
    global base_waypoints
    global waypoint_tree
    base_waypoints = waypoints
    global waypoints_2d
    if not waypoints_2d:
        waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        waypoint_tree = KDTree(waypoints_2d)
def publish_waypoints(closest_idx):
    # fill in final waypoints to publish
    lane = Lane()
    lane.header = base_waypoints.header
    lane.waypoints = base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

    # fill in path for visulization in Rviz
    path = Path()
    path.header.frame_id = '/odom'
    for p in lane.waypoints:
        path_element = PoseStamped()
        path_element.pose.position.x = p.pose.pose.position.x
        path_element.pose.position.y = p.pose.pose.position.y
        path.poses.append(path_element)

    final_waypoints_pub.publish(lane)
    final_path_pub.publish(path)
def get_closest_waypoint_idx():
    x = pose.pose.position.x
    y = pose.pose.position.y

    closest_idx = waypoint_tree.query([x,y],1)[1]

    return closest_idx

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/dakebot/center_pose', PoseStamped,pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, waypoints_cb)
        final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        final_path_pub = rospy.Publisher('/final_path', Path, queue_size=1)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if pose and base_waypoints and waypoint_tree:
                # Get closest waypoint
                closest_waypoint_idx = get_closest_waypoint_idx()
                publish_waypoints(closest_waypoint_idx)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
