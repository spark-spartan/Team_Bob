#!/usr/bin/env python

import rospy
import random

def detFun():

    # psuedo detection
    waitTime = random.randint(1,5)
    rospy.loginfo( 'Detection for %s' % waitTime + ' seconds')
    rate = rospy.Rate(1./waitTime) #hz
    rate.sleep()
    
    success = random.randint(0,4) # 80% chance of success
    if success:
        rospy.loginfo( 'detection succeeded' )
        return 'success'
    elif not(success):
        rospy.loginfo( 'detection failed' )
        return 'failure'


