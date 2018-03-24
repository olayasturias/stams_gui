#!/usr/bin/env python


import roslib
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np
import time



if __name__ == '__main__':

    rospy.init_node("depth_publisher", log_level=rospy.DEBUG)
    rospy.loginfo("Starting node depth_publisher")

    # Read parameters from console (o launch file)
    i = 0

    # Create TransformStamped message
    transf = geometry_msgs.msg.TransformStamped()
    transf.transform.translation.x = 0 
    transf.transform.translation.y = 0
    transf.transform.translation.z = 0
    transf.header.frame_id = 'world'
    transf.child_frame_id = 'sonar'
    transf.transform.rotation.x = 0.0
    transf.transform.rotation.y = 0.0
    transf.transform.rotation.z = 0.0
    transf.transform.rotation.w = 1.0

    br = tf2_ros.TransformBroadcaster()


    while not rospy.is_shutdown():
        rospy.sleep(1)
        transf.header.stamp = rospy.Time.now()
        transf.transform.translation.z = i
        br.sendTransform(transf)
        i += 1

