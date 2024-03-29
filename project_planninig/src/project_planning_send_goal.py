#!/usr/bin/env python3
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received


def feedback_callback(feedback):

    print('[Feedback] Going to Goal Pose...')


# initializes the action client node
rospy.init_node('move_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.pose.position.x = 0.0145
goal.target_pose.pose.position.y = 0.3431
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = -0.012642
goal.target_pose.pose.orientation.w = 0.99992

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
# time.sleep(3.0)
# client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time
# status = client.get_state()
# check the client API link below for more info

client.wait_for_result()

print('[Result] State: %d' % (client.get_state()))
