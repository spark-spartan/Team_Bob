#!/usr/bin/env python

import rospy

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from strands_executive_msgs.srv import AddTasks, DemandTask, SetExecutionStatus

def get_service(service_name, service_type):    
    rospy.loginfo('Waiting for %s service...' % service_name)
    rospy.wait_for_service(service_name)
    rospy.loginfo("Done")        
    return rospy.ServiceProxy(service_name, service_type)

def get_execution_status_service():
    return get_service('/task_executor/set_execution_status', SetExecutionStatus)

def get_demand_task_service():
    return get_service('/task_executor/demand_task', DemandTask)
    
def get_add_tasks_service():
    return get_service('/task_executor/add_tasks', AddTasks)

# MAIN
if __name__ == '__main__':
    rospy.init_node('myAction')

    task = Task()
    task.action = '/wait_action'
    max_wait_seconds = 15 *60
    task.max_duration = rospy.Duration(max_wait_seconds)
    
    wpls = ['ChargingPoint' , 'Station' ,
            'WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4', 'WayPoint5', 'WayPoint6',
            'WayPoint7', 'WayPoint8', 'WayPoint9', 'WayPoint10', 'WayPoint11', 'WayPoint12',
            'WayPoint13', 'WayPoint14', 'WayPoint15', 'WayPoint16', 'WayPoint17', 'WayPoint18',
            'WayPoint19'] 
            
    task.start_after = rospy.get_rostime() + rospy.Duration(10)
    task.end_before = task.start_after + rospy.Duration(task.max_duration.to_sec() * (len(wpls)+1))

    task_utils.add_time_argument(task, rospy.Time())
    task_utils.add_duration_argument(task, rospy.Duration(2))

    #wpls = ['WayPoint4','WayPoint5','WayPoint6','WayPoint7'] 
     
    set_execution_status = get_execution_status_service()
    set_execution_status(True)
    
    tasks = []#saeed
    
    for i in range(len(wpls)):
        task.start_node_id = wpls[i]
        task.end_node_id =  wpls[i]
        add_tasks = get_add_tasks_service()
        add_tasks(task)
            
        #tasks.append(task)#saeed
    
    #indent saeed
    #demand_task = get_demand_task_service()
    #demand_task(task)
    
    

