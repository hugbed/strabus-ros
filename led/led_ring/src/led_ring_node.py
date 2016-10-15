#!/usr/bin/env python

import rospy

from led_node.msg import Command
from led_node.msg import Config

# Callback used for commands received from the led_ring_command topic.
def commandCallback(command):
    rospy.loginfo(rospy.get_caller_id() + ": Received %s command for LED id %d" % (command.command, command.id)
    
# Callback used for configuration received from the led_ring_config topic.
def configCallback(config):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', config.config)

# Main node function.
if __name__ == '__main__':
    rospy.init_node("led_ring_node", anonymous=True)

    rospy.Subscriber("led_ring_command", Command, commandCallback)
    rospy.Subscriber("led_ring_config", Config, configCallback)

    rospy.spin()