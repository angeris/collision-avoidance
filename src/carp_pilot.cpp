/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : carp_pilot.cpp
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Aug 18, 2020.
  Description  : ros quad pilot using CARP
**************************************************************************/

#include<carp_ros/carp_pilot.h>
#include<iostream>

CarpPilot::CarpPilot() {
    // retrieve ROS parameter
    ros::param::param<std::string>(
      "targetPose_topic", targetPose_topic, "command/pose"); 

    ros::param::param<std::string>(
      "obstacleList_topic", obstacleList_topic, "obstacleList");

    
    // ROS subs and pub
    targetPose_sub = nh_.subscribe<geometry_msgs::Pose>(
      targetPose_topic, 1, &CarpPilot::targetPose_CB, this);
    
    ROS_INFO_STREAM("Subscribed: "<< targetPose_topic);

    obstacleList_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      obstacleList_topic, 10);

    // inital pose is pose after takeoff
    targetPoseSp.pose.position.x = takeoffPose_.pose.position.x;
    targetPoseSp.pose.position.y = takeoffPose_.pose.position.y;
    targetPoseSp.pose.position.z = takeoffHeight_;
}


void CarpPilot::targetPose_CB(
        const geometry_msgs::Pose::ConstPtr& msg) {
    // unpapack the pose
    targetPoseSp.pose = *msg;
}

void CarpPilot::controlLoop() {
    targetPoseSp.header.stamp = ros::Time::now();
    px4SetPosPub_.publish(targetPoseSp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "default_controller_node");
    return 0;
  }
