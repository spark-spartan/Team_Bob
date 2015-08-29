/**
 * @file      motion_planning.cpp
 * @brief     Main motion planning node
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-08
 * @copyright (MIT) 2015 Edinferno
 */

#include <navigation/monitored_nav_wrapper.hpp>
#include <navigation/topo_nav_wrapper.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh("navigation");

  ROS_INFO("Running Navigation");
  MonitoredNavWrapper monitored_nav_wrapper(&nh);
  TopoNavWrapper topo_nav_wrapper(&nh);

  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
