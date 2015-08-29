#!/usr/bin/env python
import rospy
import actionlib
import mary_tts.msg
import mary_tts.srv
import os
import re
import string
from random import randint


class Speak:
    speak_file = ('speak.csv')

    def robot_speak(self):
        rospy.init_node('robot_speech', anonymous=True)
        r = rospy.Rate(0.1)  # 0.1Hz
        regex = re.compile('[%s]' % re.escape(string.punctuation))
        try:
            s = rospy.ServiceProxy('/mary_tts/set_voice', mary_tts.srv.ros_mary)
            s.wait_for_service()
            
        except rospy.ServiceException:
            print 'service exception'
        
        #add a file containing list of words etc
        while True:

            while not rospy.is_shutdown():
                
                rndint = randint(0, self.file_len)
                with open(self.speak_file) as f:
                    line = list(f)[rndint]
                    line = regex.sub('', str(line))
                    print 'speaking phrase: ', line

                maryclient = actionlib.SimpleActionClient('speak', mary_tts.msg.maryttsAction)
                maryclient.wait_for_server()
                print 'client response'
                speak = mary_tts.msg.maryttsGoal()
                speak.text = line
                maryclient.send_goal_and_wait(speak)

                r.sleep()

            break

    def __init__(self):
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        os.chdir("..")
        os.chdir("files")
        with open(self.speak_file) as f:
            for line, l in enumerate(f):
                pass
            self.file_len = line
            self.robot_speak()

if __name__ == '__main__':
    try:
        Speak()
    except rospy.ROSInterruptException:
        pass
