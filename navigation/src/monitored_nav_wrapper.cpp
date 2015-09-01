/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include <navigation/monitored_nav_wrapper.hpp>

MonitoredNavWrapper::MonitoredNavWrapper(ros::NodeHandle* nh) : nh_(nh),
  topological_nav_ac("topological_navigation", true),
  monitored_nav_ac("monitored_navigation", true) {
  this->loadParams();
  this->init();
  this->rosSetup();
}

MonitoredNavWrapper::~MonitoredNavWrapper() {
}

void MonitoredNavWrapper::loadParams() {
  ros::param::param("personal_space", personal_space_, 1.2f);
  ros::param::param("approach_timeout", approach_timeout_, 5.0f);
  ros::param::param("node_nav_timeout", node_nav_timeout_, 5.0f);
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
  ROS_INFO("Waiting for Topological Nav Action Server...");
  topological_nav_ac.waitForServer();
  ROS_INFO("Topological Nav Action Server ready!");
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

  topological_navigation::GotoNodeGoal goal;
  goal.target = req.waypoint;

  topological_nav_ac.sendGoal(goal);

  bool finished_before_timeout =
    topological_nav_ac.waitForResult(ros::Duration(node_nav_timeout_));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = topological_nav_ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    return true;
  } else {
    ROS_INFO("Action did not finish before the time out.");
    return false;
  }
}

bool MonitoredNavWrapper::startHumanApproach(
  navigation::StartHumanApproach::Request& req,
  navigation::StartHumanApproach::Response& res) {

  strands_navigation_msgs::MonitoredNavigationGoal goal;

  goal.action_server = "move_base";
  goal.target_pose = person_pose_;

  monitored_nav_ac.sendGoal(goal);

  bool finished_before_timeout =
    monitored_nav_ac.waitForResult(ros::Duration(approach_timeout_));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = monitored_nav_ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    return true;
  } else {
    ROS_INFO("Action did not finish before the time out.");
    return false;
  }
}
