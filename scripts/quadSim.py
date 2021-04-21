#!/usr/bin/env python
# math
import numpy as np
import numpy.random as rand
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped


class Quad(object):

    def __init__(self):
        rospy.init_node('quadsim', anonymous=True)
        self.position = rospy.get_param("~position").split(",")
        self.state_pub = rospy.Publisher('mavros/local_position/pose',
                                         PoseStamped,
                                         queue_size=10)
        self.vision_pub = rospy.Publisher('mavros/vision_pose/pose',
                                          PoseStamped,
                                          queue_size=10)

        rospy.loginfo("quadsim: Initalization complete")

    def update(self):
        poseSP = PoseStamped()
        poseSP.pose.position.x = float(self.position[0]) + rand.random()*.1
        poseSP.pose.position.y = float(self.position[1]) + rand.random()*.1
        poseSP.pose.position.z = float(self.position[2]) + rand.random()*.1
        poseSP.header.stamp = rospy.Time.now()
        poseSP.header.frame_id = '/world'
        return poseSP

    def run(self):
        msg = self.update()
        self.state_pub.publish(msg)
        self.vision_pub.publish(msg)
        rospy.sleep(.01)


if __name__ == '__main__':
    quad = Quad()
    while not rospy.is_shutdown():
        quad.run()
