/*
* @Author: GreatAlexander
* @Date:   2015-08-29
* @Last Modified by:   GreatAlexander
* @Last Modified time: 2015-08-29
*/

// #include <iostream>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <wait_action/WaitAction.h>
#include <strands_executive_msgs/Task.h>
#include <mongodb_store_msgs/StringPair.h>
// #include <strands_executive_msgs/task_utils.h> // Only Python
#include <strands_executive_msgs/AddTasks.h>
#include <strands_executive_msgs/SetExecutionStatus.h>
#include <strands_executive_msgs/DemandTask.h>

using namespace std;
typedef actionlib::SimpleActionClient<wait_action::WaitAction> Client;

int main(int argc, char** argv) {

  ros::init(argc, argv, "exec_tutorial");
  ros::NodeHandle nh;
  Client client("do_dishes", true); // true -> don't need ros::spin()
  ROS_INFO("Running exec_tutorial");
  // MoveToAction move_to(nh, "move_to");
  size_t max_wait_minutes = 60 * 60;

  strands_executive_msgs::Task task;
  task.action = "wait_action";
  ros::Duration max_duration(max_wait_minutes);
  task.max_duration = max_duration;

  // Set Time
  double ros_time = ros::Time().toSec();
  mongodb_store_msgs::StringPair time;
  time.first = strands_executive_msgs::Task::TIME_TYPE;
  time.second = std::to_string(ros_time);

  task.arguments.push_back(time);

  // Set Duration
  double ros_duration = ros::Duration(10).toSec();
  mongodb_store_msgs::StringPair duration;
  duration.first = strands_executive_msgs::Task::DURATION_TYPE;
  duration.second = std::to_string(ros_duration);

  task.arguments.push_back(duration);

  // Start After
  task.start_after = ros::Time() + ros::Duration(10);

  // End Before
  task.end_before = task.start_after + ros::Duration(task.max_duration.toSec() *
                                                     3);

  // Set Execution Status service
  ros::ServiceClient set_execution_status;
  set_execution_status =
    nh.serviceClient<strands_executive_msgs::SetExecutionStatus>(
      "/task_executor/set_execution_status");
  set_execution_status.waitForExistence();

  strands_executive_msgs::SetExecutionStatus set_execution_srv;
  set_execution_srv.request.status = true;

  set_execution_status.call(set_execution_srv);

  // Demand Task service
  ros::ServiceClient demand_task;
  demand_task =
    nh.serviceClient<strands_executive_msgs::DemandTask>(
      "/task_executor/demand_task");
  demand_task.waitForExistence();

  strands_executive_msgs::DemandTask demand_task_srv;
  demand_task_srv.request.task = task;

  demand_task.call(demand_task_srv);

  // Add Task service
  ros::ServiceClient add_tasks;
  add_tasks =
    nh.serviceClient<strands_executive_msgs::AddTasks>(
      "/task_executor/add_tasks");
  add_tasks.waitForExistence();

  strands_executive_msgs::AddTasks add_tasks_srv;
  add_tasks_srv.request.tasks.push_back(task);

  add_tasks.call(add_tasks_srv);

  ros::spin();

  ros::shutdown();

  return 0;
}
