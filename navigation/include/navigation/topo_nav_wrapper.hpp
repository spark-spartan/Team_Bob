/**
 * @file      motion_planning.cpp
 * @brief     Main motion topological planning node
 * @author    William Blyth + Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */


#ifndef TOPO_NAV_WRAPPER_HPP
#define TOPO_NAV_WRAPPER_HPP

#include <ros/ros.h>

class TopoNavWrapper {
 public:
  TopoNavWrapper(ros::NodeHandle* nh);
  ~TopoNavWrapper();

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

#endif  /* TOPO_NAV_WRAPPER_HPP */
