/**
 * @file      motion_planning.cpp
 * @brief     Main topological motion planning node
 * @author    William Blyth + Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include <navigation/topo_nav_wrapper.hpp>

TopoNavWrapper::TopoNavWrapper(ros::NodeHandle* nh) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

TopoNavWrapper::~TopoNavWrapper() {
}

void TopoNavWrapper::loadParams() {
}

void TopoNavWrapper::init() {
}

void TopoNavWrapper::rosSetup() {
}
