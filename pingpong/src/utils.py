## Waiting for the user's input

import rospy
import threading
import actionlib

def waitForInput(actionClient):
    rospy.loginfo('listening to the user''s input:\nc=cancell all goals\np=cancel goal\nq=quit')

    while not rospy.is_shutdown():
        input = raw_input()
        if len(input)>0:
            rospy.loginfo("received: %s as an input",input)
        if input=='p':
            actionClient.cancel_goal()  
        elif input=='c':
            actionClient.cancel_all_goals()
 
        elif input=='q':
            rospy.loginfo('shutting down...')
            rospy.signal_shutdown("just because")
    