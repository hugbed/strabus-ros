#! /usr/bin/env python

# ROS packages.
import rospy
import actionlib

# Stepper generated messages.
import Stepper.msg

# Controller library.
import L6470 from L6470

class MoveToAction(object):
    # Create messages that are used to publish feedback/result
    _feedback = Stepper.msg.MoveToFeedback()
    _result = Stepper.msg.MoveToResult()

    def __init__(self, device):
        # Setup action server
        self._action_name = "MoveTo"
        self._as = actionlib.SimpleActionServer(self._action_name, Stepper.msg.MoveToAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # Configure Stepper controller
        self._controller = L6470()
        self._controller.open(0, device)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # Publish info to the console for the user.
        rospy.loginfo('%s: Ordering stepper to target position %i' % (self._action_name, goal.targetPos))

        # TODO: implement moveTo
        
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
    def shutdown(self):
        self._controller.close()

class RunAction(object):
    # Create messages that are used to publish feedback/result
    _feedback = Stepper.msg.MoveToFeedback()
    _result = Stepper.msg.MoveToResult()

    def __init__(self, device):
        # Setup action server
        self._action_name = "Run"
        self._as = actionlib.SimpleActionServer(self._action_name, Stepper.msg.RunAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # Configure Stepper controller
        self._controller = L6470()
        self._controller.open(0, device)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # Publish info to the console for the user.
        rospy.loginfo('%s: Ordering stepper to run in direction %s' % (self._action_name, goal.direction))

        # TODO: implement run
        
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
    def shutdown(self):
        self._controller.close()
