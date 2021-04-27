/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : carp_pilot.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Aug 18, 2020.
  Description  : ros quad pilot using CARP
**************************************************************************/

#ifndef __CARP_PILOT_POLY_H__
#define __CARP_PILOT_POLY_H__

#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "carp_ros/CarpServicePoly.h"

class CarpPilot {
 public:
    CarpPilot();
    ~CarpPilot();
    // quad name
    std::string quadID_;
    // topic name: estimation 
    std::string obstacleList_topic_;
    // topic name: command goal
    std::string targetGoal_topic_;
    // topic names: setpoint pose and twist
    std::string targetPose_topic_;
    std::string targetTwist_topic_;
    // service name: main carp service
    std::string carpService_topic_;


 private:
    // ros node handle 
    ros::NodeHandle nh_;
    // safe pose and twist target calculated by carpService
    geometry_msgs::PoseStamped targetPoseSp_;
    geometry_msgs::TwistStamped targetTwistSp_;
    // list of obstacles returned by the estimator
    carp_ros::obstacleArray obstacleList_;

    // timer loops
    // setpoint loop
    double setpointFreq_;
    ros::Timer setpointTimer_; 
    void setpointLoopCB(const ros::TimerEvent& event);
    // planner loop
    double plannerFreq_;
    ros::Timer plannerTimer_; 
    void plannerLoopCB(const ros::TimerEvent& event);
    // estimation loop
    double estimationFreq_;
    ros::Timer estimationTimer_; 
    void estimationLoopCB(const ros::TimerEvent& event);

    // px4 pose subscriber
    // current pose
    geometry_msgs::PoseStamped currentPose_;
    ros::Subscriber currentPose_sub;
    void currentPose_CB(const geometry_msgs::PoseStamped& msg);

    // current vel
    geometry_msgs::TwistStamped currentTwist_;
    ros::Subscriber currentTwist_sub;
    void currentTwist_CB(const geometry_msgs::TwistStamped& msg);


    // target pose
    geometry_msgs::Pose goalPose_;
    ros::Subscriber targetGoal_sub_; 
    void targetGoal_CB(const geometry_msgs::Pose& msg);

    // ob list subscriber
    ros::Subscriber obstacleList_sub_;
    void obstacle_CB(const carp_ros::obstacleArray& msg);

    // out going pubs
    ros::Publisher targetPose_pub;
    ros::Publisher targetTwist_pub;
    // current trajectory time
    // ros::Time trajectoryTime_=0.0f;

    // carp service
    carp_ros::CarpServicePolyt carpSrv_; //data container
    ros::ServiceClient carpServiceClient_; //service client

};

#endif