#!/usr/bin/env python3  
import rospy
import math
import tf2_ros
import tf
import csv
def pose_distance(p1,p2):
    return math.sqrt(math.pow((p2[0] - p1[0]), 2) + math.pow((p2[1] - p1[1]), 2))

def writeposition(row):
    with open('/home/wangdak/catkin_ws/src/function/points/waypoints.csv','a') as f:
        writer = csv.writer(f)
        writer.writerow(row)

if __name__ == '__main__':
    point=[0,0]
    lastpoint=[0,0]
    distance=0
    count=0
    rospy.init_node('robot_pose_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue 
        x=trans.transform.translation.x
        y=trans.transform.translation.y
        quanternion=[trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        euler = tf.transformations.euler_from_quaternion(quanternion)
        yaw = euler[2]
        if count==0:
            lastpoint[0]=x
            lastpoint[1]=y
            count=1
        point[0]=x
        point[1]=y
        distance=pose_distance(point,lastpoint)
        # 当距离大于0.1时保存一次路径点
        if distance>0.1:
            s=str(x)+' '+str(y)+'\n'
            row=[x,y,yaw]
            writeposition(row)
            lastpoint[0]=x
            lastpoint[1]=y
        rate.sleep()
