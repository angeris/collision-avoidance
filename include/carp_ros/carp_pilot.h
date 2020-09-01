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

#include <mslquad/px4_base_controller.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "carp_ros/EllipsoidArray.h"
#include "carp_ros/CarpService.h"

class CarpPilot : public PX4BaseController {
 public:
    CarpPilot();
    // topic name: estimation 
    std::string obstacleList_topic_;
    // topic name: command pose
    std::string targetPose_topic_;
    // service name: main carp service
    std::string carpService_topic_;


 protected:
    // goal pose send by the user
    geometry_msgs::Pose goalPose_;
    // safe pose target calculated by carpService
    geometry_msgs::PoseStamped targetPoseSp_;
    // list of obstacles returned by the estimator
    carp_ros::EllipsoidArray obstacleList_;
    // main control loop
    void controlLoop() override;

 private:
    // px4 pose subscriber
    ros::Subscriber targetPose_sub_; 
    void targetPose_CB(const geometry_msgs::Pose::ConstPtr& msg);

    // ob list subscriber
    ros::Subscriber obstacleList_sub_;
    void obstacle_CB(const carp_ros::EllipsoidArray::ConstPtr& msg);

    // carp service
    carp_ros::CarpService carpSrv_; //data container
    ros::ServiceClient carpServiceClient_; //service client

};

#endif