/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include <navigation/monitored_nav_wrapper.hpp>

MonitoredNavWrapper::MonitoredNavWrapper(ros::NodeHandle* nh) : nh_(nh),
  monitored_nav_ac("monitored_navigation", true) {
  this->loadParams();
  this->init();
  this->rosSetup();
}

MonitoredNavWrapper::~MonitoredNavWrapper() {
}

void MonitoredNavWrapper::loadParams() {
  ros::param::param("personal_space", personal_space_, 1.2f);
  ros::param::set("/move_base/DWAPlannerROS/xy_goal_tolerance", personal_space_);
}

void MonitoredNavWrapper::init() {
}

void MonitoredNavWrapper::rosSetup() {
  person_pose_sub_ = nh_->subscribe("/people_tracker/pose", 1,
                                    &MonitoredNavWrapper::personPoseCB, this);
  monitored_feedback_sub_ = nh_->subscribe("/monitored_navigation/feedback", 1,
                                           &MonitoredNavWrapper::feedbackCB, this);
  start_human_approach_ =
    nh_->advertiseService("start_human_approach",
                          &MonitoredNavWrapper::startHumanApproach, this);
  ROS_INFO("Waiting for Monitored Nav Action Server...");
  monitored_nav_ac.waitForServer();
  ROS_INFO("Monitored Nav Action Server ready!");
}

void MonitoredNavWrapper::personPoseCB(const
                                       geometry_msgs::PoseStamped::ConstPtr& msg) {
  person_pose_ = *msg;
}

void MonitoredNavWrapper::feedbackCB(const
                                     strands_navigation_msgs::
                                     MonitoredNavigationActionFeedback::
                                     ConstPtr& msg) {
  // person_pose_ = *msg;
}

bool MonitoredNavWrapper::startHumanApproach(
  navigation::StartHumanApproach::Request& req,
  navigation::StartHumanApproach::Response& res) {
  res.ok = true;
  // person_pose_ = req.person_pose;
  this->sendNavGoal();
  return true;
}

void MonitoredNavWrapper::sendNavGoal() {
  strands_navigation_msgs::MonitoredNavigationGoal goal;
  goal.action_server = "monitored_navigation";
  goal.target_pose = person_pose_;
  monitored_nav_ac.sendGoal(goal);
}
