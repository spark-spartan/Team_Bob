#!/usr/bin/env python
import rospy
import actionlib
import mary_tts.msg
import mary_tts.srv
import os
import re
import string
import speak.srv
from random import randint


class Speak:

    def speak_server(self):
        rospy.init_node('bob_speak_server', anonymous=True)
        rospy.Service('bob_speak', speak.srv.BobSpeak, self.speak_cb)
        print 'service started'
        rospy.spin()

    def change_voice(self):
        try:
            s = rospy.ServiceProxy('/ros_mary/set_voice', mary_tts.srv.SetVoice)
            s.wait_for_service()
            s('dfki-spike-hsmm')
            print 'voice changed'

        except rospy.ServiceException:
            print 'service exception while changing voice'

    def speak_cb(self, req):
        print 'request recieved'

        maryclient = actionlib.SimpleActionClient('speak', mary_tts.msg.maryttsAction)
        maryclient.wait_for_server()
        print 'mary_client response'
        line = getattr(self, req.speech_type)()
        print line 
        
        speak = mary_tts.msg.maryttsGoal()
        speak.text = line
        maryclient.send_goal_and_wait(speak)

    def __init__(self):
        print 'init'
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        os.chdir("..")
        os.chdir("files")
        self.regex = re.compile('[%s]' % re.escape(string.punctuation))

        self.change_voice()
        self.speak_server()

    def file_length(self, fname):
        with open(fname) as f:
            for line, l in enumerate(f):
                pass
            return line

    def fortune(self):
        fortune_length = self.file_length('fortune.csv')
        with open('fortune.csv') as f:
            line = list(f)[randint(0, fortune_length)]
            line = self.regex.sub('', str(line))
        return line

    def joke(self):
        joke_length = self.file_length('joke.csv')
        with open('joke.csv') as f:
            line = list(f)[randint(0, joke_length)]
            line = self.regex.sub('', str(line))
        return line

    def greeting(self):
        print 'hello world'
        greeting_length = self.file_length('greeting.csv')
        with open('greeting.csv') as f:
            line = list(f)[randint(0, greeting_length)]
            line = self.regex.sub('', str(line))
        return line

    def farewell(self):
        farewell_length = self.file_length('farwell.csv')
        with open('farwell.csv') as f:
            line = list(f)[randint(0, farewell_length)]
            line = self.regex.sub('', str(line))
        return line

if __name__ == '__main__':
    try:
        Speak()
    except rospy.ROSInterruptException:
        pass
