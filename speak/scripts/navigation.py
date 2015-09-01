#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  1 17:44:51 2015

@author: sandy
"""

import rospy
import actionlib
from speak.msg import EmptyAction
import topological_navigation.msg

class Navigation():
    def __init__(self):
        self.wpls = ['WayPoint5', 'WayPoint6']
        self.cnt = 0
        self.server = actionlib.SimpleActionServer(
            '/navigate_mentor',
            EmptyAction,
            self.execute_cb,
            False
        )
        rospy.loginfo("Creating topo nav client...")
        self.client = actionlib.SimpleActionClient(
            '/topological_navigation',
            topological_navigation.msg.GotoNodeAction
        )
        self.client.wait_for_server()
        rospy.loginfo(" ... done ")
        self.server.start()
        
    def execute_cb(self, goal):
       g =  topological_navigation.msg.GotoNodeGoal()
       g.target = self.wpls[self.cnt]
       self.client.send_goal_and_wait(g)
       self.cnt += 1
       self.cnt = self.cnt % len(self.wpls)
       self.server.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("navigate_mentor")
    n = Navigation()
    rospy.spin()
        