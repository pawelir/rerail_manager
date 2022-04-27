#! /usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Empty
import actionlib
from rerail_manager.msg import RerailRotracAction, RerailRotracFeedback, RerailRotracResult, RerailRotracGoal

def feedback_callback(feedback):
    rospy.loginfo(feedback.process_state)
    rospy.loginfo("Detected " + str(feedback.grooves_number) + " groove(s).")

def rerail_client():
    client = actionlib.SimpleActionClient('/rerail_rotrac', RerailRotracAction)                     # Initialize a client
    client.wait_for_server()                                                                        # Wait until server is activated
    goal = RerailRotracGoal()
    client.send_goal(goal,feedback_cb=feedback_callback)                                            # Send a goal to action server
    client.wait_for_result()
    return client.get_result()                                                                      # Return a result


if __name__ == '__main__':
    try:
        rospy.init_node('rerail_rotrac_client')
        result = rerail_client()
        rospy.loginfo(f'Action completed: {result.success}')                                         # Display the final result
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)