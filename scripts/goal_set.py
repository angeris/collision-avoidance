#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np


class Captian:

    def __init__(self):
        rospy.init_node('poseCaptain', anonymous=True)

        # goal topic
        self.manifest = rospy.get_param("~manifest").split(",")
        self.goalPubs = [rospy.Publisher(drone+'/command/goal',
                         Pose, queue_size=10)
                         for drone in self.manifest]
        self.goals = {drone: Pose() for drone in self.manifest}
        # wait for connect
        rospy.sleep(2)

    def run(self):

        while not rospy.is_shutdown():
            print("enter cmd")
            cmd = raw_input("u for update, g for go")
            if cmd == "u":
                self.setGoals()
            elif cmd == "g":
                self.sendGoals()
            else:
                ROS_WARN("invalid cmd")
            # publish
            self.goalPub.publish(goalMsg)
            rospy.sleep(.5)

    def setGoals(self):
        for drone in self.manifest:
            print("enter goal for " + drone)
            x, y, z = [float(val) for val in raw_input(
                "Enter goal position: ").split()]
            # save goal cmd
            self.goals[drone].position.x = x
            self.goals[drone].position.y = y
            self.goals[drone].position.z = z

    def sendGoals(self):
        for drone, pub in zip(self.manifest, self.goalPub):
            pub.publish(self.goals[drone])


if __name__ == '__main__':
    captian = Captian()
    captian.run()
