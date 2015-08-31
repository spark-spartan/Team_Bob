import rospy
import random


def navFun(wayPoint):
    
    # psuedo navigation
    waitTime = random.randint(1,5)
    msg =  'Navigating to ' + wayPoint
    msg = msg + ' for %s' % waitTime + ' seconds'
    rospy.loginfo(msg)

    rate = rospy.Rate(1./waitTime) #hz
    rate.sleep()
    
    success = random.randint(0,4) # 80% chance of success
    if success:
        rospy.loginfo( 'navigation succeeded' )
        return 'success'
    elif not(success):
        rospy.loginfo( 'navigation failed' )
        return 'failure'

