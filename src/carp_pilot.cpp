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
      "targetPose_topic", targetPose_topic_, "command/pose"); 
    ros::param::param<std::string>(
      "obstacleList_topic", obstacleList_topic_, "obstacleList");
    ros::param::param<std::string>(
      "carpService_topic", carpService_topic_, "/carpService");

    
    // ROS subs and pub
    targetPose_sub_ = nh_.subscribe<geometry_msgs::Pose>(
      targetPose_topic_, 1, &CarpPilot::targetPose_CB, this);
    ROS_INFO_STREAM("Subscribed: "<< targetPose_topic_);

    obstacleList_sub_ = nh_.subscribe<carp_ros::EllipsoidArray>(
      obstacleList_topic_, 1, &CarpPilot::obstacle_CB, this);

    // inital pose is pose after takeoff
    targetPoseSp_.pose.position.x = takeoffPose_.pose.position.x;
    targetPoseSp_.pose.position.y = takeoffPose_.pose.position.y;
    targetPoseSp_.pose.position.z = takeoffHeight_;
    // set inital goal to takeoff
    goalPose_ = targetPoseSp_.pose;
    carpSrv_.request.goal = goalPose_.position;

    // start 
    // services
    carpServiceClient_ = nh_.serviceClient<carp_ros::CarpService>(
        carpService_topic_, true);
  }


void CarpPilot::targetPose_CB(const geometry_msgs::Pose::ConstPtr& msg) {
  // unpack the pose
  ROS_INFO("goal");
  goalPose_ = *msg;
  carpSrv_.request.goal = goalPose_.position;

}

void CarpPilot::obstacle_CB(const carp_ros::EllipsoidArray::ConstPtr& msg){
  // save the list of obstacles
  ROS_INFO("obs");
  obstacleList_ = *msg;
  carpSrv_.request.obstacles = obstacleList_;
}


void CarpPilot::controlLoop() {
    // call the CARP service
    ROS_INFO("loop");
    carpSrv_.request.position = curPose_.pose.position;
    if (carpServiceClient_.call(carpSrv_)){
        ROS_INFO("goal projected");
        targetPoseSp_.pose.position = carpSrv_.response.point;
      }
    else{
      ROS_ERROR("Failed to call carp service");
      }
    // update target pose to latest 


    // publish the target pose
    targetPoseSp_.header.stamp = ros::Time::now();
    px4SetPosPub_.publish(targetPoseSp_);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "carp_pilot_node");
    ROS_INFO_STREAM("Controller Initiated");
    CarpPilot pilot;
    while(ros::ok())
    {
      ros::spin();
    }
    return 0;

  }
