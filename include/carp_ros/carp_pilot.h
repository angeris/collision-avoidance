/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : carp_pilot.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Aug 18, 2020.
  Description  : ros quad pilot using CARP
**************************************************************************/

#ifndef __CARP_PILOT_H__
#define __CARP_PILOT_H__

#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "carp_ros/CarpService.h"

class CarpPilot {
 public:
    CarpPilot();
    ~CarpPilot();
    // quad ame
    std::string quadID_;
    // topic name: estimation 
    std::string obstacleList_topic_;
    // topic name: command goal
    std::string targetGoal_topic_;
    // topic names: setpoint pose and twist
    std::string targetPose_topic_;
    std::string targetTwist_topic_:
    // service name: main carp service
    std::string carpService_topic_;


 protected:
    // ros node handle 
    ros::NodeHandle nh_;
    // goal pose send by the user
    geometry_msgs::Pose goalPose_;
    // safe pose and twist target calculated by carpService
    geometry_msgs::PoseStamped targetPoseSp_;
    geometry_msgs::TwistStamped targetTwistSp_;
    // list of obstacles returned by the estimator
    carp_ros::EllipsoidArray obstacleList_;

    // timer loops
    // setpoint loop
    double setpointFreq_;
    ros::Timer setpointTimer_; 
    void setpointLoopCB();
    // planner loop
    double plannerFreq_;
    ros::Timer plannerTimer_; 
    void plannerLoopCB();
    // estimation loop
    double estimationFreq_;
    ros::Timer estimationTimer_; 
    void estimationLoopCB();

 private:


    // px4 pose subscriber
    // current pose
    geometry_msgs::PoseStamped currentPose_;
    void currentPose_CB(const geometry_msgs::PoseStamped& msg)
    // target pose
    ros::Subscriber targetGoal_sub_; 
    void targetGoal_CB(const geometry_msgs::Pose& msg);

    // ob list subscriber
    ros::Subscriber obstacleList_sub_;
    void obstacle_CB(const carp_ros::EllipsoidArray& msg);

    // out going pubs
    ros::Publisher targetPose_pub;
    ros::Publisher targetTWist_pub;
    // current trajectory time
    // ros::Time trajectoryTime_=0.0f;

    // carp service
    carp_ros::CarpService carpSrv_; //data container
    ros::ServiceClient carpServiceClient_; //service client

};

#endif