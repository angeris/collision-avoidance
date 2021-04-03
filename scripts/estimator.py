#!/usr/bin/env python
# math
import numpy as np
import numpy.random as rand
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from carp_ros.msg import Ellipsoid, obstacleArray


class Estimator(object):

    def __init__(self):
        rospy.init_node('Estimator', anonymous=True)
        self.estimator_pub = rospy.Publisher('obstacleList',
                                             obstacleArray,
                                             queue_size=10)

        rospy.loginfo("Estimator: Initalization complete")

    def update(self):
        raise NotImplementedError

    def run(self):
        msg = self.update()
        self.estimator_pub.publish(msg)
        rospy.sleep(.3)


class StaticEstimator(Estimator):
    """docstring for ClassName"""

    def __init__(self):
        super(StaticEstimator, self).__init__()

    def update(self):
        obArray = obstacleArray()
        ellipsoidMsg = Ellipsoid()
        # fill fake ellipsoid
        ellipsoidMsg.center = [1, 0, 2] + rand.random(3)*.1
        ellipsoidMsg.shape = np.eye(3).flatten().tolist()
        # push back
        obArray.names.append("quad_x")
        obArray.ellipsoids.append(ellipsoidMsg)
        return obArray


if __name__ == '__main__':
    Estimator = StaticEstimator()
    while not rospy.is_shutdown():
        Estimator.run()