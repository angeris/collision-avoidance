#!/usr/bin/env python

'''
The MIT License (MIT)
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
'''
# std lib imports
import sys
import time
import numpy as np
import numpy.linalg as la

# Standard ROS message
import rospy
from std_msgs.msg import Bool, Time
from geometry_msgs.msg import Pose, PoseStamped, Twist

from carp_ros.msg import Ellipsoid, obstacleArray


class Estimator(object):
    """docstring for Estimator"""

    def __init__(self,):
        rospy.init_node('estimator', anonymous=True)
        self.estimator_pub = rospy.Publisher('obstacleList',
                                             obstacleArray,
                                             queue_size=10)

    def loop(self):
        obArray = obstacleArray()
        obArray.names.append("quad_x")
        E = Ellipsoid()
        E.center = [1, 0, 2]
        E.shape = np.eye(3).flatten().tolist()
        obArray.ellipsoids.append(E)

        self.estimator_pub.publish(obArray)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    # make tower
    estimator = Estimator()
    estimator.run()
