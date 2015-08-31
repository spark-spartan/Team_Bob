#!/usr/bin/env python
import rospy
import actionlib
import mary_tts.msg
import mary_tts.srv
import strands_gazing.msg
import os
import speak.srv
from random import randint


class Speak:

    def speak_server(self):
        rospy.init_node('bob_speak_server', anonymous=True)
        gaze = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        print 'hitting gaze server'
        goal = strands_gazing.msg.GazeAtPoseGoal()
        goal.topic_name = '/upper_body_detector/closest_bounding_box_centre'
        goal.runtime_sec = 0

        gaze.send_goal(goal)
        gaze.wait_for_result()
        print gaze.get_result()
        print 'server hit'

        rospy.Service('bob_speak', speak.srv.BobSpeak, self.speak_cb)
        print 'service started'

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

        speak = mary_tts.msg.maryttsGoal()
        speak.text = line
        maryclient.send_goal_and_wait(speak)
        rospy.sleep(2.)
        maryclient.cancel_all_goals()

    def __init__(self):
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        os.chdir("..")
        os.chdir("files")

        self.change_voice()
        self.speak_server()

    def file_length(self, fname):
        with open(fname) as f:
            for line, l in enumerate(f):
                pass
            return line

    def fortune(self):
        fortune_length = self.file_length('fortune.txt')
        with open('fortune.txt') as f:
            line = list(f)[randint(0, fortune_length)]
        return line

    def joke(self):
        joke_length = self.file_length('jokes.txt')
        with open('jokes.txt') as f:
            line = list(f)[randint(0, joke_length)]
        return line

    def greeting(self):
        print 'hello world'
        greeting_length = self.file_length('greeting.txt')
        with open('greeting.txt') as f:
            line = list(f)[randint(0, greeting_length)]
        return line

    def farewell(self):
        farewell_length = self.file_length('farwell.txt')
        with open('farwell.txt') as f:
            line = list(f)[randint(0, farewell_length)]
        return line

if __name__ == '__main__':
    try:
        Speak()
    except rospy.ROSInterruptException:
        pass
