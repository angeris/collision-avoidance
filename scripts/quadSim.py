#!/usr/bin/env python
# math
import numpy as np
import numpy.random as rand
# ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped


class Quad(object):

    def __init__(self):
        rospy.init_node('quadsim', anonymous=True)
        self.position = rospy.get_param("~position").split(",")
        self.pose_pub = rospy.Publisher('mavros/local_position/pose',
                                        PoseStamped,
                                        queue_size=10)
        self.vision_pub = rospy.Publisher('mavros/vision_pose/pose',
                                          PoseStamped,
                                          queue_size=10)

        self.twist_pub = rospy.Publisher('mavros/local_position/velocity',
                                         TwistStamped,
                                         queue_size=10)

        rospy.loginfo("quadsim: Initalization complete")

    def updatePose(self):
        # pose
        poseSP = PoseStamped()
        poseSP.pose.position.x = float(self.position[0]) + rand.random()*0
        poseSP.pose.position.y = float(self.position[1]) + rand.random()*0
        poseSP.pose.position.z = float(self.position[2]) + rand.random()*0
        poseSP.header.stamp = rospy.Time.now()
        poseSP.header.frame_id = '/world'
        return poseSP

    def updateVelocity(self):
        # twist
        twistSP = TwistStamped()
        twistSP.twist.linear.x = .1
        twistSP.twist.linear.y = .1
        twistSP.twist.linear.z = .1
        twistSP.header.stamp = rospy.Time.now()
        twistSP.header.frame_id = '/world'
        return twistSP

    def run(self):
        pose = self.updatePose()
        self.pose_pub.publish(pose)
        self.vision_pub.publish(pose)

        twist = self.updateVelocity()
        self.twist_pub.publish(twist)

        rospy.sleep(.01)


if __name__ == '__main__':
    quad = Quad()
    while not rospy.is_shutdown():
        quad.run()
