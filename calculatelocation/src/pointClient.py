#!/usr/bin/env python

import threading
import roslib; roslib.load_manifest('calculatelocation')
from calculatelocation.srv import *
import rospy
import random




def call_client(x,y):
    # init a node; not mandatory, but it allows using rospy.loginfo methods, and communicate with the node using ROS tools.
    rospy.init_node('myClient', anonymous=True)
    serviceName ='myLocation'

    # waiting to the service to load; this is a blocking call, but a timeout parameter can be provided.
    try:
        rospy.wait_for_service(serviceName,timeout=1)
    except rospy.ROSException as e:
        # the service is not available, an exception is caught
        rospy.logerr('%s', e)
        rospy.signal_shutdown('timeout has reached; shutting down the client')
        sys.exit(1)

    rospy.loginfo('found the service. Can continue :)')

    # call the service (base on its name) and type.
    proxy = rospy.ServiceProxy(serviceName, Location)

    # ----- Call the service using Request object ------
    # create a LocationRequest object (included in calculatelocation.srv package)
    req = LocationRequest(x, y)

    # send the Request object to the service, and receive a LocationResponse object
    respo = proxy(req)
    
    # another option to call the service, without creatin a Request object
    #respo = proxy(x, y)
    
    # prints the response (breakdown the Request object)
    rospy.loginfo('received:\nResponse class type %s\nquadrant: %s\ndistance from origin: %s',type(respo), respo.quadrant.data, respo.distance)

    # keep the node alive for a couple of second, until exiting 
    rospy.sleep(5)
    

if __name__ == '__main__':
    try:
        # define default values, in case there is no imput from the user.
        x = random.randrange(-100,100)
        y = random.randrange(-100,100)
        
        # read the arguments from the command line, is any
        if len(sys.argv)>=2:
            x = int(sys.argv[1])
        
        if len(sys.argv)==3:
            y = int(sys.argv[2])
        
        # load the client
        call_client(x ,y)
    except rospy.ROSInterruptException as e:
        rospy.logerr('error in calling the server: %s', e)
        pass