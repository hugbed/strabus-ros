#! /usr/bin/env python

# ROS packages.
import rospy
import actionlib

# Action servers
import StepperActions

if __name__ == '__main__':
    # ID of the SPI device
    device = 0

    # Init ROS node
    rospy.init_node('Stepper')

    # Init actions servers
    moveto = MoveToAction(device)
    run = RunAction(device)
    
    rospy.spin()
    
    # Shutdown servers.
    moveto.shutdown()
    run.shutdown()
