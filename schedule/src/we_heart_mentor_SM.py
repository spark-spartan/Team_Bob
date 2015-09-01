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
#import sys
#sys.path.append("../../navigation")

#from states import *
from navigation.srv import NavigateToWaypoint, NavigateToWaypointRequest
from speak.srv import BobSpeak, BobSpeakRequest

class DetectHuman():
    def __init__():
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['wayPointIdx'])
    def execute(self, userdata):
        rospy.loginfo('SMACH executing state \'Detect\'')
        return detect()
        
counter = 0


def pop_waypoint(ls):
    # Rotating the waypoint list one step to the right
    global counter
    counter += 1
    counter = counter % len(ls)
    print 'Getting waypoint from list!'
    return ls[counter]


def main():
    rospy.init_node('smach_example_state_machine')

    # List of waypoints TODO: re-order them
    wpls = ['WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4', 'WayPoint5', 'WayPoint6',
            'WayPoint7', 'WayPoint8', 'WayPoint9', 'WayPoint10', 'WayPoint11', 'WayPoint12',
            'WayPoint13', 'WayPoint14', 'WayPoint15', 'WayPoint16', 'WayPoint17', 'WayPoint18',
            'WayPoint19']

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('NAV_TO_WP',
                               smach_ros.ServiceState('navigation/navigate_to_waypoint',
                               NavigateToWaypoint,
                               request = NavigateToWaypointRequest(waypoint = pop_waypoint(wpls))),
                               transitions={'succeeded':'NAV_TO_WP'})

        smach.StateMachine.add('BOB_SPEAK',
                               smach_ros.ServiceState('speak/bob_speak'),
                               BobSpeak,
                               request = BobSpeakRequest(speech_type = 'greeting')))
                               transitions={'succeeded':'NAV_TO_WP'})
        smach.StateMachine.add('DETECT_HUMAN',
                               DetectHuman(),
                               transitions={'succeeded':'BOB_SPEAK'
                                           'failure':'NAV_TO_WP'})
        
        #smach.StateMachine.add('DETECT_PERSON', smach_ros.MonitorState("/people_tracker/pose", Empty, monitor_cb),
                           #transitions={'invalid':'DETECT_PERSON', 'valid':'APROACH_PERSON', 'preempted':'DETECT_PERSON'})

        #smach.StateMachine.add('APROACH_PERSON',
                           #smach_ros.ServiceState('navigation/navigate_to_waypoint',
                                        #NavigateToWaypoint,
                                        #request = NavigateToWaypointRequest(waypoint = pop_waypoint(wpls))),
                           #transitions={'succeeded':'NAV_TO_WP'})

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
