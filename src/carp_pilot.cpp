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
  // topic names
  ros::param::param<std::string>(
    "~targetGoal_topic", targetGoal_topic_, "command/goal"); 
  ros::param::param<std::string>(
    "~targetPose_topic", targetPose_topic_, "command/pose"); 
  ros::param::param<std::string>(
    "~targetTwist_topic", targetTwist_topic_, "command/twist"); 
  ros::param::param<std::string>(
    "~obstacleList_topic", obstacleList_topic_, "obstacleList");
  ros::param::param<std::string>(
    "~carpService_service", carpService_topic_, "carpService");

  // loop frequncies
  ros::param::param<double>(
    "~setpointFreq", setpointFreq_, 60.0);  
  ros::param::param<double>(
    "~plannerFreq", plannerFreq_, 40.0); 


  
  // ROS subs
  currentPose_sub = nh_.subscribe(
    "mavros/local_position/pose", 1,
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
  // wait for the service to _actually_ be online
  carpServiceClient_.waitForExistence();
  ROS_INFO("connected to planner service");

  // initial pose is pose after takeoff
  while (ros::ok() && currentPose_.header.seq < 10) {
      std::cout << "waiting for initial pose." << std::endl;
      ros::spinOnce();
      ros::Duration(1.0).sleep();
  }


  // while (ros::ok() && obstacleList_.ellipsoids.size() == 0) {
  //     std::cout << "waiting for obs " << std::endl;
  //     ros::spinOnce();
  //     ros::Duration(1.0).sleep();
  // }


  carpSrv_.request.position = currentPose_.pose.position;
  goalPose_.position.x = currentPose_.pose.position.x;
  goalPose_.position.y = currentPose_.pose.position.y;
  goalPose_.position.z = 1.5f;
  goalPose_.orientation = currentPose_.pose.orientation;
  carpSrv_.request.goal = goalPose_.position;
  carpSrv_.request.obstacles = obstacleList_;

   // initial service call
  ROS_INFO("calling service");
  if (carpServiceClient_.call(carpSrv_)) {
    targetPoseSp_.pose.position = carpSrv_.response.projection;
    ROS_INFO("inital planner success");
  } else{
    ROS_WARN("IDK MAN");
  }

  // start timers
  // setpointTimer_ = nh_.createTimer(
  //   ros::Duration(.01),
  //   &CarpPilot::setpointLoopCB, this);

  plannerTimer_ = nh_.createTimer(
    ros::Duration(.01),
    &CarpPilot::plannerLoopCB, this);
 
  // estimationTimer_ = nh_.createTimer(
  //   ros::Duration(1.0/estimationFreq_),
  //   &CarpPilot::estimationLoopCB, this);
  ROS_INFO("pilot: initialzation complete");
}

CarpPilot::~CarpPilot(){
  ROS_INFO("pilot: destroyed");

}

void CarpPilot::currentPose_CB(const geometry_msgs::PoseStamped& msg){
  currentPose_ = msg;
}

void CarpPilot::targetGoal_CB(const geometry_msgs::Pose& msg) {
  // unpack the pose
  ROS_INFO("new goal");
  goalPose_ = msg;
}

void CarpPilot::obstacle_CB(const carp_ros::obstacleArray& msg){
  // save the list of obstacles
  obstacleList_ = msg;
}


void CarpPilot::setpointLoopCB(const ros::TimerEvent& event) {
  // step along trajectory  
  // publish the target pose
  double delay = event.current_real.toSec() - event.current_expected.toSec();
  // std::cout << delay << std::endl;
  targetPoseSp_.header.stamp = ros::Time::now();
  // targetTwistSp_.header.stamp = ros::Time::now();
  targetPose_pub.publish(targetPoseSp_);
  // targetTwist_pub.publish(targetTwistSp_)
}

void CarpPilot::plannerLoopCB(const ros::TimerEvent& event) {
  // call the CARP service
  carpSrv_.request.position = currentPose_.pose.position;
  carpSrv_.request.goal = goalPose_.position;
  carpSrv_.request.obstacles = obstacleList_;

  double startTime = ros::Time::now().toSec();
  if (carpServiceClient_.call(carpSrv_)){
      targetPoseSp_.pose.position = carpSrv_.response.projection;
      targetPoseSp_.pose.orientation.x = 0.;
      targetPoseSp_.pose.orientation.y = 0.;
      targetPoseSp_.pose.orientation.z = 0.;
      targetPoseSp_.pose.orientation.w = 1.;
  } else{
    ROS_ERROR("Failed to call carp service");
  }
  double delay = ros::Time::now().toSec() - startTime;

  targetPoseSp_.header.stamp = ros::Time::now();
  // targetTwistSp_.header.stamp = ros::Time::now();
  targetPose_pub.publish(targetPoseSp_);
  // std::cout << delay << std::endl;
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
