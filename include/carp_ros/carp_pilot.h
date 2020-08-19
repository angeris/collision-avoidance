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

class CarpPilot : public PX4BaseController {
 public:
    CarpPilot();
    // topic name for the carp planner
    std::string obstacleList_topic;
    // topic name for the target pose
    std::string targetPose_topic;

 protected:
    geometry_msgs::PoseWithCovarianceStamped obstacleList;
    geometry_msgs::PoseStamped targetPoseSp;
    void controlLoop() override;

 private:
    ros::Subscriber targetPose_sub;  // px4 pose subscriber
    ros::Publisher obstacleList_pub; // ob list publisher
    void targetPose_CB(const geometry_msgs::Pose::ConstPtr& msg);
};

#endif