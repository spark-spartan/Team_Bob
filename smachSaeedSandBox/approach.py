import rospy
import random

def appFun():

    # psuedo approaching
    waitTime = random.randint(1,5)
    rospy.loginfo( 'Approaching for %s' % waitTime + ' seconds')
    rate = rospy.Rate(1./waitTime) #hz
    rate.sleep()
    
    success = random.randint(0,4) # 80% chance of success
    if success:
        rospy.loginfo( 'Approaching succeeded' )
        return 'success'
    elif not(success):
        rospy.loginfo( 'Approaching failed' )
        return 'failure'


