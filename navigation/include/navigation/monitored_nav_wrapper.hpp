/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */


#ifndef MONITORED_NAV_WRAPPER_HPP
#define MONITORED_NAV_WRAPPER_HPP

#include <ros/ros.h>

class MonitoredNavWrapper {
 public:
  MonitoredNavWrapper(ros::NodeHandle* nh);
  ~MonitoredNavWrapper();

  void loadParams();
  void init();
  void rosSetup();

 private:
  // Flags

  // Constants

  // Variables

  // ROS
  ros::NodeHandle* nh_;

};

#endif  /* MONITORED_NAV_WRAPPER_HPP */
