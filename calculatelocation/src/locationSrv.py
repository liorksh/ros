#!/usr/bin/env python
## A node that creates a Service nodes

import threading
from std_msgs.msg import String
import roslib; roslib.load_manifest('calculatelocation')
from calculatelocation.srv import *
import rospy
import math

def run_service():
    # define the service: name, messaged and a callback method.
    pub = rospy.Service('myLocation', Location, service_callback)

    # the name of the node is static, there be only one isstance of a service.
    rospy.init_node('calculateLocationNode')

    rospy.loginfo('starting the Calculate Location service')

    rospy.spin()
    

def service_callback(req):
    quadrant = 'not found'
    rospy.loginfo('Received request to calculate: %s, %s', req.x, req.y)
    if req.x==0 and req.y==0:
        quadrant = 'origin'
    elif req.x<0:
        if req.y<0:
            quadrant = 'third'
        else:
            quadrant = 'second'
    else:
        if req.y<0:
            quadrant = 'third'
        else:
            quadrant = 'first'

    # create an instance of String object
    result = String()
    result.data = quadrant
    # calculate the distance from the origin
    distanceFromOrigin = math.sqrt(req.x**2 + req.y**2)

    # create a Response message and return to the server
    return LocationResponse(quadrant=result, distance=distanceFromOrigin)

if __name__ == '__main__':
    try:
        # call a methos to load the service
        run_service()
    except rospy.ROSInterruptException:
        pass
