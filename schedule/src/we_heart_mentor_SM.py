#!/usr/bin/env python
"""
Created by:
    TEAM:  WE <3 MENTOR
    gatufella@gmail.com (Mariela De Lucas) 2015.08.30
    
Description:
    Simple state machine for fortune-teller Bob robot

Usage:
    $> ./we_heart_mentor_SM.py
"""

import rospy
import smach
import smach_ros

from states import *
from smach_ros import ServiceState

def pop_waypoint(ls):
    # Rotating the waypoint list one step to the right
    ls.appendleft(ls.pop())
    return ls[1]

def main():
    rospy.init_node('smach_example_state_machine')
    
    # List of waypoints TODO: re-order them
    wpls = ['WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4', 'WayPoint5', 'WayPoint6',
            'WayPoint7', 'WayPoint8', 'WayPoint9', 'WayPoint10', 'WayPoint11', 'WayPoint12',
            'WayPoint13', 'WayPoint14', 'WayPoint15', 'WayPoint16', 'WayPoint17', 'WayPoint18',
            'WayPoint19']
            
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted'])

    # Open the container
    with sm:
        # Add states to the container
        
        smach.StateMachine.add('NAV_TO_WP', 
                                ServiceState('navigation/navigate_to_waypoint',
                                NavigateToWaypoint,
                                request_cb = navigate_request,
                                result_cb = navigate_result,
                                input_keys = [pop_waypoint(wpls)]),
                                transitions={'succeeded':'DETECT_PERSON'})

                           
        #smach.StateMachine.add('FOO', ExampleState(), {'done':'BAR'})
        #smach.StateMachine.add('BAR', ExampleState(), {'done':'BAZ'})
        #smach.StateMachine.add('BAZ',
                               #ExampleState(),
                               #{'done':'succeeded'})
	
	#Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
