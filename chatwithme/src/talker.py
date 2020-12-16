#!/usr/bin/env python
## A simple node that publishes std_msgs/Strings messages to the 'chatwithme' topic

import rospy
import utils
import threading
from std_msgs.msg import String

def talker():
    # the topic message is String (not the Python string, but the std_msg String)
    # the queue size is the number of messages that will be retained before starting to delete (in case there are no subscribers).
    pub = rospy.Publisher('chatwithme', String, queue_size=5)
    rospy.init_node('my_talker', anonymous=True)

    rospy.loginfo('starting the talker node, enter "q" to exit')

    rate = rospy.Rate(1) # in hz

    while not rospy.is_shutdown():
        message = "Hi from {}, the time is {}".format(rospy.get_caller_id(),rospy.get_time())
        rospy.loginfo("sent a message to %s subscribers" % pub.get_num_connections())
        pub.publish(message)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        threading.Thread(target=utils.waitForInput).start()
        talker()
    except rospy.ROSInterruptException:
        pass
