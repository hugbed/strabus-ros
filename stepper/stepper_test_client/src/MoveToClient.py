#! /usr/bin/env python

# ROS packages.
import rospy
import actionlib

from __future__ import print_function

# Stepper generated messages.
import Stepper.msg

def MoveToClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveToAction) to the constructor.
    client = actionlib.SimpleActionClient('Stepper', Stepper.msg.MoveToAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = Stepper.msg.MoveToGoal(target=100)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    # return client.get_result()  # A MoveToResult (not implemented for now)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('StepperClient')
        result = MoveToClient()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)