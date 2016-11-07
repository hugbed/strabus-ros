#! /usr/bin/env python

# ROS packages.
import rospy

# Stepper generated messages.
from std_msg.msg import String

# Controller library.
from L6470_pkg.L6470_lib import L6470

# JSON tool
import json

# Callback used to translate the received JSON message to a stepper command.
# Expecting something like this (example for the run command):
# {
#    "command" : "run",
#    "parameters" : [
#        {
#            "direction" : "clockwise",
#            "speed" : 800.0
#        }
#    ]
#}
def messageCallback(message):
    # Unpack JSON message
    data = json.loads(message.data)
    
    # Execute the given command with its parameters.
    command = data['command']
    if command == "run":
        parameters = data['parameters'][0]
        runCommand(parameters['direction'], parameters['speed'])
    elif command == "move":
        parameters = data['parameters'][0]
        moveCommand(parameters['direction'], parameters['steps'])
    elif command == "goTo":
        parameters = data['parameters'][0]
        goToCommand(parameters['position'])
    elif command == "stop":
        parameters = data['parameters'][0]
        stopCommand(parameters['type'])
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized command \"%s\"" % (command))


# Run the stepper in the given direction and at the given speed.
# The stepper will run until a stop command is issued or until a limit switch is hit.
def runCommand(direction, speed):
    if (direction == "clockwise" or direction == "counter-clockwise"):
        rospy.loginfo(rospy.get_caller_id() + ": Run at %d step/s in %s rotation" % (speed, direction))
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized run direction for \"%s\"" % (direction))
    
    # Run the stepper if the direction is appropriate.
    if (direction == "clockwise"):
        _controller.run(L6470.DIR_CLOCKWISE, speed)
    elif (run.direction == "counter-clockwise"):
        _controller.run(L6470.DIR_COUNTER_CLOCKWISE, speed)
    
# Move the stepper by the given number of microsteps.
def moveCommand(direction, steps):
    if (direction == "clockwise" or direction == "counter-clockwise"):
        rospy.loginfo(rospy.get_caller_id() + ": Move by %d steps in %s rotation" % (steps, direction))
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized move direction for \"%s\"" % (direction))
    
    # Issue a Move command to the stepper if the direction is appropriate.
    if (direction == "clockwise"):
        _controller.move(L6470.DIR_CLOCKWISE, steps)
    elif (direction == "counter-clockwise"):
        _controller.move(L6470.DIR_COUNTER_CLOCKWISE, steps)

# Move the stepper to the given position.
def goToCommand(position):
    rospy.loginfo(rospy.get_caller_id() + ": Go to position %d" % (position))

    # Issue a GoTo command to the stepper.
    _controller.goTo(position)
    
# Stop the stepper, either immediately or after a deceleration curve.
def stopCommand(type):
    if (type == "hard" or type == "soft"):
        rospy.loginfo(rospy.get_caller_id() + ": %s stop" % (type))
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized stop type for \"%s\"" % (type))

    # Issue a stop command to the stepper.
    if (type == "hard"):
        _controller.hardStop()
    elif (type == "soft"):
        _controller.softStop()

# Main node function.
if __name__ == '__main__':
    # Open SPI controller.
    _controller = L6470()
    _controller.open(0, 0)

    rospy.init_node("stepper_controller_node", anonymous=True)
    rospy.Subscriber("stepper_controller_topic", String, messageCallback)

    rospy.spin()
    
    _controller.close()
