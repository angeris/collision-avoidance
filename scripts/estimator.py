#!/usr/bin/env python
# math
import numpy as np
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from carp_ros.msg import Ellipsoid, EllipsoidArray


class Estimator(object):

    def __init__(self):
        rospy.init_node('Estimator', anonymous=True)

        self.outputTopic = "quad0/obstacleList"

        # path goal and position goal topics
        self.trajPub = rospy.Publisher(
            self.outputTopic, EllipsoidArray, queue_size=10)

        rospy.loginfo("Estimator: Initalization complete")

    def update(self):
        raise NotImplementedError

    def run(self):
        ellipsoidArrayMsg = self.update()
        self.trajPub.publish(ellipsoidArrayMsg)
        rospy.sleep(.3)


class StaticEstimator(Estimator):
    """docstring for ClassName"""

    def __init__(self):
        super(StaticEstimator, self).__init__()

    def update(self):
        ellipsoidArrayMsg = EllipsoidArray()
        ellipsoidMsg = Ellipsoid()
        # fill fake ellipsoid
        ellipsoidMsg.center.x = 1.
        ellipsoidMsg.center.y = 1.
        ellipsoidMsg.center.z = 2.
        ellipsoidMsg.shape = np.eye(3).flatten()
        # push back
        ellipsoidArrayMsg.ellipsoids.append(ellipsoidMsg)
        return ellipsoidArrayMsg


if __name__ == '__main__':
    Estimator = StaticEstimator()
    while not rospy.is_shutdown():
        Estimator.run()
