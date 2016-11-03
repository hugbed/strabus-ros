#!/usr/bin/env python

# Core ROS libs
import rospy

# Controller lib
from PCA9530_pkg.PCA9530_lib import PCA9530

# ROS messages
from led_controller.msg import Command
from led_controller.msg import Config

# Callback used for commands received from the led_controller_command topic.
def commandCallback(command):
    if command.id == 0:
        idStr = "LED0"
    elif command.id == 1:
        idStr = "LED1"
    elif command.id == 2:
        idStr = "LED0 and LED1"
    else:
        rospy.logwarn(rospy.get_caller_id() + " Unrecognized LED identifier \"%d\"" % (command.id))
        return

    if command.command == "on":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"on\" command on %s" % (idStr))
        _controller.ledOn(command.id)
    elif command.command == "off":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"off\" command on %s" % (idStr))
        _controller.ledOff(command.id)
    elif command.command == "blink0":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"blink0\" command on %s" % (idStr))
        _controller.ledBlink0(command.id)
    elif command.command == "blink1":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"blink1\" command on %s" % (idStr))
        _controller.ledBlink1(command.id)
    else:
        rospy.logwarn(rospy.get_caller_id() + ": Unsupported command \"%s\"" % (command.command))
        return

# Callback used for configuration received from the led_controller_config topic.
def configCallback(config):
    if config.id == 0:
        idStr = "BLINK0"
    elif config.id == 1:
        idStr = "BLINK1"
    else:
        rospy.logwarn(rospy.get_caller_id() + " Unrecognized BLINK identifier \"%d\"" % (config.id))
        return

    if config.config == "duty-cycle":
        rospy.loginfo(rospy.get_caller_id() + ": Configuring duty cycle for %s" % (idStr))
        _controller.ledOn(command.id)
    elif config.config == "period":
        rospy.loginfo(rospy.get_caller_id() + ": Configuring period for %s" % (idStr))
        _controller.ledOff(command.id)
    else:
        rospy.logwarn(rospy.get_caller_id() + ": Unsupported command \"%s\"" % (command.command))
        return

# Main node function.
if __name__ == '__main__':
    rospy.init_node("led_controller_node", anonymous=True)

   # _controller = PCA9530()
   # _controller.open(0)

    rospy.Subscriber("led_controller_command", Command, commandCallback)
    rospy.Subscriber("led_controller_config", Config, configCallback)

    rospy.spin()
