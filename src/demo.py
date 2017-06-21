#! /usr/bin/env python
# Olaya Alvarez Tunon

import rospy

from nav_msgs import Odometry
from geometry_msgs import Pose
import tf

def VelocityPublisher():
    pub = rospyPublisher('velocity publish', nav_msgs, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():


def PoseCallback():


def Navigation_Demo():
    rospy.init_node('Demo_navigation', anonymous = True)

    rospy.Subscriber('/g500/Pose',geometry_msgs, PoseCallback)

    VelocityPublisher()


if __name__ == '__main__':

