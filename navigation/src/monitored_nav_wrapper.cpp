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
  navigate_to_waypoint_ =
    nh_->advertiseService("navigate_to_waypoint",
                          &MonitoredNavWrapper::navigateToWaypoint, this);
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

bool MonitoredNavWrapper::navigateToWaypoint(
  navigation::NavigateToWaypoint::Request& req,
  navigation::NavigateToWaypoint::Response& res) {
  // person_pose_ = req.person_pose;
  waypoint = req.waypoint;
  this->sendNavGoal(true);
  res.res = true;
  return true;
}

bool MonitoredNavWrapper::startHumanApproach(
  navigation::StartHumanApproach::Request& req,
  navigation::StartHumanApproach::Response& res) {
  // person_pose_ = req.person_pose;
  this->sendNavGoal(false);
  res.res = true;
  return true;
}

void MonitoredNavWrapper::sendNavGoal(bool waypoint_nav) {
  strands_navigation_msgs::MonitoredNavigationGoal goal;
  if (waypoint_nav) {
    goal.action_server = waypoint;
  } else {
    goal.action_server = "move_base";
    goal.target_pose = person_pose_;
  }
  monitored_nav_ac.sendGoal(goal);
}
