#!/usr/bin/env python3  
import rospy
import tf2_ros
import tf
import numpy as np
from math import cos, sin
from geometry_msgs.msg import PoseStamped, TwistStamped

from gazebo_msgs.msg import ModelStates, LinkStates

if __name__ == '__main__':
    rospy.init_node('robot_pose_listener')
    center_pose_pub = rospy.Publisher('/dakebot/center_pose', PoseStamped, queue_size = 1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        vehicle_position=PoseStamped()
        vehicle_position.pose.position = trans.transform.translation
        vehicle_position.pose.orientation = trans.transform.rotation
        orientation = trans.transform.rotation
        (_, _, yaw)=tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
        time_stamp = rospy.Time.now()
        # vehicle center position
        center_pose = PoseStamped()
        center_pose.header.frame_id = '/odom'
        center_pose.header.stamp = time_stamp
        center_pose.pose.position = vehicle_position.pose.position
        center_pose.pose.orientation = vehicle_position.pose.orientation
        center_pose_pub.publish(center_pose)
        rate.sleep()
