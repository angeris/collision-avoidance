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
from filter import FilterBounded3D as setFilter


class setEstimator(Estimator):
    """docstring for Estimator"""

    def __init__(self):
        # ROS INIT
        super(setEstimator, self).__init__()
        # get params
        self.egoName = rospy.get_namespace().strip("/")
        self.manifest = rospy.get_param("~manifest").split(",")
        self.manifest.remove(self.egoName)
        self.nDrones = len(self.manifest)
        self.poseTopic = "/mavros/vision_pose/pose"
        # set up subs and wait for initial measurements 
        self.poses = {}
        self.ellipsoids = {k: Ellipsoid() for k in self.manifest}
        self.subs = []
        for drone in self.manifest:
            self.subs.append(rospy.Subscriber(
                                "/"+drone+self.poseTopic,
                                PoseStamped,
                                self.poseCB, drone))
        while len(self.poses) != self.nDrones:
            rospy.loginfo("waiting for intial data")
            rospy.sleep(1)
        # estimation  parameters
        initNoise = np.eye(3)  # 1 meter initial error
        pNoise = np.eye(3)*0.001  # .1 m every .01 sec (10 m/s)
        mNoise = np.eye(3)*0.09  # .3 localization error

        # z elongated margin
        self.bubble = np.array([[.1, 0, 0],
                                [0, .1, 0],
                                [0, 0, .4]])

        # init estimation objects
        self.filters = {k: setFilter(self.poses[k],
                                     initNoise, pNoise, mNoise)
                        for k in self.manifest}
        rospy.loginfo("setEstimator: Initalization Complete")

    def poseCB(self, msg, key):
        pose = [msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z]
        self.poses[key] = np.array(pose)

    def updateEstimates(self):
        for name, pose in self.poses.items():
            # take noisy measurement
            measurement = self.filters[name].measure(pose)
            self.ellipsoids[name].center = list(measurement)
            # update the estimate
            try:
                self.filters[name].update(measurement)
                # add margin to the estimate
                shape = self.filters[name].addMargin(self.bubble, typ='elip')
                # update outgoing objects
                self.ellipsoids[name].shape = shape.flatten().tolist()
            except np.linalg.LinAlgError:
                rospy.logwarn("failed to update shape, using last known")

    def update(self):
        obArray = obstacleArray()
        self.updateEstimates()
        for name, elip in self.ellipsoids.items():
            obArray.names.append(name)
            obArray.ellipsoids.append(elip)

        return obArray


if __name__ == '__main__':
    # make tower
    estimator = setEstimator()
    while not rospy.is_shutdown():
        estimator.run()
