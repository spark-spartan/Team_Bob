import rospy
import numpy as np
from bayes_people_tracker.msg import PeopleTracker


def detect():
    rospy.Subscriber('/people_tracker/people', PeopleTracker, detect_cb)


def detect_cb(people):
    if people.min_distance < 3 and people.min_distance > 0 and people.min_distance_angle > -np.pi / 4 and people.min_distance_angle < np.pi / 4:
        return 'success'
    else:
        return 'failure'
