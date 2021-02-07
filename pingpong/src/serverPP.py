#!/usr/bin/env python

import rospy
import actionlib
import threading
import random

# import all the messages in the current package
from pingpong.msg import *

class PingPongServer:
    ALLOW_NEW_GOALS = 1
    ALLOW_PREEMPT   = 2
    _serverFlags = 0

    def __init__(self, allowInterruptions):
        self.serviceName = 'myPingPongGame'
        self._serverFlags = allowInterruptions
    def run_service(self):

        # the name of the node is static, there be only one isstance of a service.
        rospy.init_node('pingpongServerNode')
        rospy.loginfo('Starting the PingPong game server')

        # define the service: name, Action message,  and a callback method.
        self.actionServer = actionlib.SimpleActionServer(self.serviceName, 
            PingPongGameAction, # this is a master message that contains metadata and information about the other messages
            execute_cb=self.action_execution, 
            auto_start=False)
        
        # start the server
        self.actionServer.start()

        # keep the serevr alive
        rospy.spin()
        
    # this method is the Goal feedback
    def action_execution(self,goal):

        rospy.loginfo('Received the game''s score: %s',goal.maxScore)
        startTime = rospy.get_rostime()
        # define a Feedback object
        feedback = PingPongGameFeedback()
        gameOn = True

        while gameOn:
            if self._serverFlags & self.ALLOW_NEW_GOALS and self.actionServer.is_new_goal_available():
                # receiving a new goal triggers also a 'is_preempt_requested', therefore is should be handled first.
                goal = self.actionServer.accept_new_goal()
                # reset the previous feedback object
                feedback = PingPongGameFeedback()
                rospy.loginfo('Received a new score: %s',goal.maxScore)
            elif self._serverFlags & self.ALLOW_PREEMPT and self.actionServer.is_preempt_requested():
                rospy.loginfo('received preempt request')
                # changes the status of the server, otherwise the server will be exited with an error
                # since there's no return value and a preempt request was received.
                # You can comment this line and see the error that is received on the client
                self.actionServer.set_preempted()
                gameOn = False
           
            elif (feedback.playerClient>=goal.maxScore) or (feedback.playerServer>=goal.maxScore):
                # if the goal was achieved - end the loop.
                gameOn = False
            else: 
                # publish the score and continue for another round
                self.actionServer.publish_feedback(feedback)
                feedback.playerClient += random.randrange(1,4)
                feedback.playerServer += random.randrange(1,4)           
                rospy.sleep(1)
    
        #rospy.loginfo('self.actionServer.preempt_request value is %s', self.actionServer.preempt_request)
        if self._serverFlags==0 or self.actionServer.preempt_request==False:
            endTime = rospy.get_rostime()

            gameDuration = endTime-startTime
        
            rospy.sleep(1)
            if feedback.playerServer==feedback.playerClient:
                winner = "tie"
            elif feedback.playerServer>feedback.playerClient:
                winner = "Server"
            else:
                winner = "Client"

            # define a Result object
            #result = pingpong.msg.PingPongGameResult
            result = PingPongGameResult()
            result.gameDuration.data = gameDuration
            result.winner.data = winner
            result.score = [feedback.playerClient, feedback.playerServer]

            # publish the result message
            self.actionServer.set_succeeded(result)       


if __name__ == '__main__':
    try:
        # initiate the flag whether the server accepts new goals or cancellation messages
        if len(sys.argv)==2:
            serverFlags = int(sys.argv[1])
        else:
            serverFlags = 0

        # call a methos to load the service
        server = PingPongServer(serverFlags)
        server.run_service()
    except rospy.ROSInterruptException:
        pass
