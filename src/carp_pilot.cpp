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
    "quad_ns", quadID_, "quad0"); 
  ros::param::param<std::string>(
    "targetGoal", targetGoal_topic_, "command/goal"); 
  ros::param::param<std::string>(
    "targetPose", targetPose_topic_, "command/pose"); 
  ros::param::param<std::string>(
    "targetTwist", targetTwist_topic_, "command/twist"); 
  ros::param::param<std::string>(
    "obstacleList", obstacleList_topic_, "obstacleList");
  ros::param::param<std::string>(
    "carpService", carpService_topic_, "carpService");

  
  // ROS subs
  currentPose_sub = nh_.subscribe(
    quadID_+"/mavros/local_position_pose", 1,
    &CarpPilot::currentPose_CB, this);

  targetGoal_sub_ = nh_.subscribe(
    targetGoal_topic_,1,
    &CarpPilot::targetGoal_CB, this);

  obstacleList_sub_ = nh_.subscribe(
    obstacleList_topic_, 1,
    &CarpPilot::obstacle_CB, this);

  //ROS pubs
  targetPose_pub = nh_.advertise<geometry_msgs::PoseStamped>(
      targetPose_topic_, 1);
  targetTwist_pub = nh_.advertise<geometry_msgs::TwistStamped>(
      targetTwist_topic_, 1);



  // services
  carpServiceClient_ = nh_.serviceClient<carp_ros::CarpService>(
      carpService_topic_, true);

  // initial pose is pose after takeoff
  // while (ros::ok() && currentPose_.header.seq < 100) {
  //     std::cout << quadNS_ << ": Waiting for initial pose." << std::endl;
  //     ros::spinOnce();
  //     ros::Duration(1.0).sleep();
  //   }
    
  goalPose_ = currentPose_.pose;
  carpSrv_.request.goal = goalPose_.position;

  // start timers

  setpointTimer_ = nh_.createTimer(
    ros::Duration(1.0/setpointFreq_),
    &CarpPilot::setpointLoopCB, this);

  // plannerTimer_ = nh_.createTimer(
  //   ros::Duration(1.0/plannerFreq_),
  //   &CarpPilot::plannerLoopCB, this);

  // estimationTimer_ = nh_.createTimer(
  //   ros::Duration(1.0/estimationFreq_),
  //   &CarpPilot::estimationLoopCB, this);



}
CarpPilot::~CarpPilot(){

}

void CarpPilot::currentPose_CB(const geometry_msgs::PoseStamped& msg){
  currentPose_ = msg;
}

void CarpPilot::targetGoal_CB(const geometry_msgs::Pose& msg) {
  // unpack the pose
  goalPose_ = msg;
  carpSrv_.request.goal = goalPose_.position;

}

void CarpPilot::obstacle_CB(const carp_ros::obstacleArray& msg){
  // save the list of obstacles
  obstacleList_ = msg;
  carpSrv_.request.obstacles = obstacleList_;

}


void CarpPilot::setpointLoopCB(const ros::TimerEvent& event) {
  // step along trajectory  
  // publish the target pose
  targetPoseSp_.header.stamp = ros::Time::now();
  // targetTwistSp_.header.stamp = ros::Time::now();
  targetPose_pub.publish(targetPoseSp_);
  // targetTwist_pub.publish(targetTwistSp_)
}

void CarpPilot::plannerLoopCB(const ros::TimerEvent& event) {
  // call the CARP service
  if (carpServiceClient_.call(carpSrv_)){
      ROS_INFO("goal projected");
      targetPoseSp_.pose.position = carpSrv_.response.projection;
  } else{
    ROS_ERROR("Failed to call carp service");
  }
}

void CarpPilot::estimationLoopCB(const ros::TimerEvent& event) {

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
