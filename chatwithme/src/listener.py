#!/usr/bin/env python

## A simple node that listens to std_msgs/Strings published to the 'chatwithme' topic

import rospy
import threading
import utils
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' received: %s', data.data)

def listener():
        
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('my_listener', anonymous=True)

    rospy.loginfo('starting the listener node, enter "q" to exit')
    
    rospy.Subscriber('chatwithme', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    threading.Thread(target=utils.waitForInput).start()
    listener()

