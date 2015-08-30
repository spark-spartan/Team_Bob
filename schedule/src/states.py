#!/usr/bin/env python

"""
Created by:
    TEAM:  WE <3 MENTOR
    gatufella@gmail.com (Mariela De Lucas) 2015.08.30

Description:
    States for fortune-teller Bob.
"""

import rospy

import smach
import smach_ros

from  navigation import NavigateToWaypoint

class NAV_TO_WP(smach.ServiceState):
    def __init__(self):
        smach.ServiceState.__init__(self, outcomes = ['done'])
        
        #__init__(self, cb, outcomes=[], input_keys=[], output_keys=[], io_keys=[]) 
    
    @smach.cb_interface(input_keys=[pop_waypoint(wpls)])    
    def navigate_request_cb(userdata, request):
       navigate_request = NavigateToWaypoint().Request       
       navigate.request.waypoint = 'i have to give a string here'
       return navigate_request
    
    def navigate_result_cb(userdata, status, result):
       if status == GoalStatus.SUCCEEDED:
          userdata.gripper_output = result.num_iterations
          return 'my_outcome'
