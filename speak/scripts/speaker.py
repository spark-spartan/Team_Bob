#!/usr/bin/env python
import rospy
import actionlib
import mary_tts.msg
import mary_tts.srv
import strands_gazing.msg
import os
import speak.srv
import thread
from std_msgs.msg import String
from random import randint


class Speak:

    def speak_server(self):
        rospy.init_node('bob_speak_server', anonymous=True)
        rospy.Service('bob_speak', speak.srv.BobSpeak, self.speak_cb)
        print 'service started'
	rospy.spin()	

    def track_human():
        gaze = actionlib.SimpleActionClient('gaze_at_pose', strands_gazing.msg.GazeAtPoseAction)
        gaze.wait_for_server()
        print 'hitting gaze server'
        goal = strands_gazing.msg.GazeAtPoseGoal()
        goal.topic_name = '/upper_body_detector/closest_bounding_box_centre'
        goal.runtime_sec = 0
        gaze.send_goal(goal)
        print 'server hit'
        
    def talk(self, line):
        speak = mary_tts.msg.maryttsGoal()
        speak.text = line
        self.maryclient.send_goal_and_wait(speak)
        self.maryclient.cancel_all_goals()
        
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

        self.maryclient = actionlib.SimpleActionClient('speak', mary_tts.msg.maryttsAction)
        self.maryclient.wait_for_server()
        print 'mary_client response'
        
        if req.speech_type == 'greeting':
            thread.start_new_thread(self.greeting, ())
            self.card_command()            
        
        else:
            getattr(self, req.speech_type)()

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
        self.talk(line)

    def joke(self):
        joke_length = self.file_length('jokes.txt')
        with open('jokes.txt') as f:
            line = list(f)[randint(0, joke_length)]
        self.talk(line)

    def greeting(self):
        greeting_length = self.file_length('greeting.txt')
        with open('greeting.txt') as f:
            line = list(f)[randint(0, greeting_length)]
        self.talk(line)

    def farewell(self):
        farewell_length = self.file_length('farewell.txt')
        with open('farewell.txt') as f:
            line = list(f)[randint(0, farewell_length)]
        self.talk(line)
    
    def card_command(self):
        rospy.Subscriber('/socialCardReader/commands', String, self.card_callback)
        
    def card_callback(self, msg):
        if msg == 'PATROL':
            self.fortune()
        elif msg == 'PAUSE_WALK':
            self.joke()
            
        self.farewell()
        
if __name__ == '__main__':
    try:
        Speak()
    except rospy.ROSInterruptException:
        pass
