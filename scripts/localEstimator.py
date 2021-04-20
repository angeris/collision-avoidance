#!/usr/bin/env python
# math
import numpy as np
import numpy.random as rand
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from carp_ros.msg import Ellipsoid, obstacleArray
from estimator import Estimator


class localEstimator(Estimator):
    """docstring for Estimator"""

    def __init__(self):
        # ROS INIT
        super(localEstimator, self).__init__()
        # get params
        self.egoName = rospy.get_namespace().strip("/")
        self.manifest = rospy.get_param("~manifest").split(",")
        self.manifest.remove(self.egoName)
        self.poseTopic = "/mavros/vision_pose/pose"
        # init estimation objects
        self.ellipsoids = {k: Ellipsoid() for k in self.manifest}
        self.subs = []
        for drone in self.manifest:
            self.subs.append(rospy.Subscriber(
                                "/"+drone+self.poseTopic,
                                PoseStamped,
                                self.poseCB, drone))

    def poseCB(self, msg, key):
        measurement = [msg.pose.position.x,
                       msg.pose.position.y,
                       msg.pose.position.z]
        self.updateEstimate(key, measurement)

    def updateEstimate(self, key, measurement):
        self.ellipsoids[key].center = measurement
        self.ellipsoids[key].shape = np.eye(3).flatten().tolist()

    def update(self):
        obArray = obstacleArray()
        for key, elip in self.ellipsoids.items():
            obArray.names.append(key)
            obArray.ellipsoids.append(elip)

        return obArray


if __name__ == '__main__':
    # make tower
    estimator = localEstimator()
    rospy.loginfo("Estimator: Initalization complete")
    while not rospy.is_shutdown():
        estimator.run()
