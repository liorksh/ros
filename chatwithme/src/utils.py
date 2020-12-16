## Waiting for the user's input

import rospy
import threading

def waitForInput():
    while not rospy.is_shutdown():
        input = raw_input()
        rospy.loginfo("received: %s as an input",input)
        if input=="q":
            rospy.loginfo('shutting down...')
            rospy.signal_shutdown("just because")
    