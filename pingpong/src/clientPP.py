import rospy
import actionlib
import threading
import utils

from pingpong.msg import * 

class PingPongClient:
    
    def __init__(self):
        self.serverName = 'myPingPongGame'

    def run_client(self, maxScore):

        # the name of the node is static, there be only one isstance of a service.
        rospy.init_node('pingpongClientNode', anonymous=True)
        rospy.loginfo('Starting the PingPong game client')

        # define the service: name, Action message.
        self.actionClient = actionlib.SimpleActionClient(self.serverName,
            # the PingPongGameAction is a master message that contains metadata and information about the other messages
            pingpong.msg.PingPongGameAction)

        # start a thread to lesten to user's inputs (receives the actionClient as a parameter)
        userInputThread = threading.Thread(target=utils.waitForInput, args=[self.actionClient])
        userInputThread.start()

        # wait for the server to be active
        serverOn = self.actionClient.wait_for_server(timeout=rospy.Duration(secs=2))
        if serverOn==False:
            rospy.logerr('The server %s is not active; shutting down the node', self.serverName)
            rospy.signal_shutdown('the server is not available')

        rospy.loginfo('The server was found')

        # create a goal
        goal = PingPongGameGoal()
        goal.maxScore = maxScore

        # initiate the client with all the callback methods
        self.actionClient.send_goal(goal,
            active_cb=self.action_active,
            done_cb=self.action_returnGoal,
            feedback_cb=self.action_returnFeedback)
        #actionClient.send_goal_and_wait()

        self.actionClient.wait_for_result()
        
        # another way to get the results
        #result = self.actionClient.get_result()
        rospy.sleep(0.5)
        rospy.loginfo('Press enter to end the client')
        if rospy.is_shutdown()==False:
            rospy.signal_shutdown('')
        
    def action_active(self):
        rospy.loginfo('The Goal was registered')

    def action_returnGoal(self,statusInt, result):
       
        # see the status legend: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        rospy.loginfo('Received status: %d, Client stats: %s', statusInt,  self.actionClient.get_state())
        if statusInt==actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Game ended:\nWinner: %s, Game duration: %d.%d, Client: [%s], Server[%s]', 
                result.winner.data, 
                result.gameDuration.data.secs,result.gameDuration.data.nsecs/10000,
                result.score[0], result.score[1])           
        elif statusInt==actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn('This goal of this client was aborted')

    def action_returnFeedback(self, score):
        rospy.loginfo('Current Score: Client[%s], Server[%s]', score.playerClient, score.playerServer)

if __name__ == '__main__':
    try:
        maxScore = 10
        if len(sys.argv)==2:
            maxScore = int(sys.argv[1])
        
        # call a methos to load the service
        client = PingPongClient()
        client.run_client(maxScore)
    except rospy.ROSInterruptException:
        pass
