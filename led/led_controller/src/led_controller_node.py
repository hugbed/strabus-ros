#!/usr/bin/env python

# Core ROS libs
import rospy

# Controller lib
from PCA9530_pkg.PCA9530_lib import PCA9530

# Import ROS standard messages.
from std_msgs.msg import String

# IMPORTANT #
# See the node's README file for further details on what parameters the node expects.

# Callback used to translate the received JSON message to an LED controller command.
# Expecting something like this (example for the LED0 on command):
# {
#     "action" : "command",
#     "parameters" : {
#         "id" : "LED0",
#         "command" : "on"
#     }
# }
def messageCallback(message):
    try:
        # Unpack JSON message
        data = json.loads(message.data)
        
        # Execute the given action with its parameters.
        action = data['action']
        if action == "command":
            parameters = data['parameters']
            command(parameters['id'], parameters['command'])
        elif action == "config":
            parameters = data['parameters']
            command(parameters['id'], parameters['config'], parameters['value'])
        else:
            rospy.logerr(rospy.get_caller_id() + ": Unrecognized action \"%s\"" % (action))
    except ValueError as e:
        rospy.logerr(rospy.get_caller_id() + ": Error decoding JSON \"%s\"" % (str(e)))

# Issue a command to the LED controller.
def command(id, command):
    if id == "LED0":
        ledID = PCA9530.LED0
    elif id == "LED1":
        ledID = PCA9530.LED1
    elif id == "BOTH":
        ledID = PCA9530.LED_BOTH
    else:
        rospy.logwarn(rospy.get_caller_id() + " Unrecognized LED identifier \"%s\"" % (id))
        return

    if command == "on":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"on\" command on %s" % (id))
        _controller.ledOn(ledID)
    elif command == "off":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"off\" command on %s" % (id))
        _controller.ledOff(ledID)
    elif command == "blink0":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"blink0\" command on %s" % (id))
        _controller.ledBlink0(ledID)
    elif command == "blink1":
        rospy.loginfo(rospy.get_caller_id() + ": Issuing \"blink1\" command on %s" % (id))
        _controller.ledBlink1(ledID)
    else:
        rospy.logwarn(rospy.get_caller_id() + ": Unsupported command \"%s\"" % (command))
        return

# Configure a parameter of the LED controller.
def config(id, config, value):
    if id == "BLINK0":
        blinkID = PCA9530.BLINK0
    elif id == "BLINK1":
        blinkID = PCA9530.BLINK1
    else:
        rospy.logwarn(rospy.get_caller_id() + " Unrecognized BLINK identifier: \"%s\"" % (id))
        return

    if config == "duty-cycle":
        rospy.loginfo(rospy.get_caller_id() + ": Configuring duty cycle for %s to %d" % (id, value))
        _controller.setBlinkDutyCycle(blinkID, value)
    elif config.config == "period":
        rospy.loginfo(rospy.get_caller_id() + ": Configuring period for %s to %d" % (id, value))
        _controller.setBlinkPeriod(blinkID, value)
    else:
        rospy.logwarn(rospy.get_caller_id() + ": Unsupported configuration \"%s\"" % (config))
        return

# Main node function.
if __name__ == '__main__':
    _controller = PCA9530()
    _controller.open(0)

    rospy.init_node("led_controller_node", anonymous=True)
    rospy.Subscriber("action", String, messageCallback)

    rospy.spin()
    
    _controller.close()
