/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include <navigation/monitored_nav_wrapper.hpp>

MonitoredNavWrapper::MonitoredNavWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

MonitoredNavWrapper::~MonitoredNavWrapper() {
}

void MonitoredNavWrapper::loadParams() {
}

void MonitoredNavWrapper::init() {
}

void MonitoredNavWrapper::rosSetup() {
}
