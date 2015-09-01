#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time 

import numpy as np

from navigation import navFun
from approach import appFun
from detection import detFun
from interaction import intFun

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~ Global Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
waypoints = ['WayPoint1', 'WayPoint2', 'WayPoint3', 'WayPoint4',
             'WayPoint5', 'WayPoint6', 'WayPoint7', 'WayPoint8',
             'WayPoint9', 'WayPoint10', 'WayPoint11', 'WayPoint12',
             'WayPoint13', 'WayPoint14', 'WayPoint15', 'WayPoint16',
             'WayPoint17', 'WayPoint18', 'WayPoint19']

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Classes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

class ChangeDestination(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success'],
                             input_keys=['wayPointIdx_in'],
                             output_keys=['wayPointIdx_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChangeDestination')
        idx = userdata.wayPointIdx_in + 1
        userdata.wayPointIdx_out = np.mod(idx, len(waypoints))
        return 'success'

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','failure'],
                             input_keys=['wayPointIdx'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation')
        rospy.loginfo('Destination = %i' %userdata.wayPointIdx)
        return navFun( waypoints[ userdata.wayPointIdx ])

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

class DetectPerson (smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['wayPointIdx'])

    def execute(self, userdata):
        rospy.loginfo('SMACH executing state \'Detection\'')
        return detFun()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

class ApproachPerson (smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['wayPointIdx'])

    def execute(self, userdata):
        rospy.loginfo('SMACH executing state \'Approach\'')
        return appFun()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

class Interact (smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'failure'],
                             input_keys=['wayPointIdx'])
        
    def execute(self, userdata):
        rospy.loginfo('SMACH executing state \'Interact\'')
        return intFun()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

def main():
    rospy.init_node('smachSaeedSandBox')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])
    sm.userdata.smWayPointIdx = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('ChangeDestination', ChangeDestination(), 
                               transitions={'success':'Navigation'},
                               remapping={'wayPointIdx_in':'smWayPointIdx', 
                                          'wayPointIdx_out':'smWayPointIdx'})
        smach.StateMachine.add('Navigation', Navigation(), 
                               transitions={'success':'DetectPerson',
                                            'failure':'ChangeDestination'},
                               remapping={'wayPointIdx':'smWayPointIdx'})

        smach.StateMachine.add('DetectPerson', DetectPerson(), 
                               transitions={'success':'ApproachPerson',
                                            'failure':'ChangeDestination'},
                               remapping={'wayPointIdx':'smWayPointIdx'})

        smach.StateMachine.add('ApproachPerson', ApproachPerson(),
                               transitions={'success':'Interact',
                                            'failure':'ChangeDestination'},
                               remapping={'wayPointIdx':'smWayPointIdx'})

        smach.StateMachine.add('Interact', Interact(),
                               transitions={'success':'ChangeDestination',
                                            'failure':'ChangeDestination'},
                               remapping={'wayPointIdx':'smWayPointIdx'})
        smach.StateMachine.add('DETECT', smach_ros.MonitorState())

    # Execute SMACH plan
    outcome = sm.execute()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

if __name__ == '__main__':
    main()
