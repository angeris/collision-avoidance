#!/usr/bin/env python
# math
import numpy as np
import numpy.linalg as la
import numpy.random as rand
import cvxpy as cvx
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from carp_ros.srv import CarpService, CarpServiceRequest, CarpServiceResponse
# lib
from carpPy.Carp import Carp


class CarpServer(object):
    """docstring for Carp"""
    def __init__(self, serviceName='carpService'):
        rospy.init_node('carpService', anonymous=True)
        self.service = rospy.Service(serviceName,
                                     CarpService,
                                     self.handleService)

        self.carp = Carp()
        rospy.loginfo("CARP service online")

    def handleService(self, req):
        rsp = CarpServiceResponse()
        # unpack request
        position = np.array([req.position.x, req.position.y, req.position.z])
        goal = np.array([req.goal.x, req.goal.y, req.goal.z])
        obstacles = [(np.array(ob.center), np.array(ob.shape))
                     for ob in req.obstacles.ellipsoids]

        try:
            projection, _ = self.carp.project(position, goal, obstacles)
            # pack response
            rsp.projection.x = projection[0]
            rsp.projection.y = projection[1]
            rsp.projection.z = projection[2]
            rsp.success = True
        except RuntimeError as e:
            rospy.logwarn("projection failure: ", e)
            rsp.projection = req.position
            resp.success = False

        return rsp


if __name__ == '__main__':
    service = CarpServer()
    while not rospy.is_shutdown():
        rospy.spin()
