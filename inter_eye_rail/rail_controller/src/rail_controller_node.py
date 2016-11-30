#! /usr/bin/env python

# ROS packages.
import rospy

# Standard messages.
from std_msgs.msg import String

# Controller library.
from L6470_pkg.L6470_lib import L6470

# JSON tool
import json

# Callback used to translate the received JSON message to a rail command.
# Expecting something like this (example for the move command):
# {
#    "command" : "move",
#    "parameters" : {
#        "direction" : "open",
#        "speed" : 800.0
#    }
#}
def messageCallback(message):
    try:
        # Unpack JSON message
        data = json.loads(message.data)

        # Execute the given command with its parameters.
        command = data.get('command', None)
        parameters = data['parameters']
        if command == "run":
            runCommand(parameters['direction'], parameters['speed'])
        elif command == "move":
            moveCommand(parameters['direction'], parameters['steps'])
        elif command == "goTo":
            goToCommand(parameters['position'])
        elif command == "stop":
            stopCommand(parameters['type'])
        else:
            rospy.logerr(rospy.get_caller_id() + ": Unrecognized command \"%s\"" % (command))
    except ValueError as e:
        rospy.logerr(rospy.get_caller_id() + ": Error decoding JSON \"%s\"" % (str(e)))


# Move command
# Move the rail at the given speed.
# The rail will move until a stop command is issued or until a limit switch is hit.
#
# speed: the speed at which the rail will move, in µm/s
def moveCommand(speed):
    if (speed > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail at %d µm/s" % (speed))
        direction = L6470.DIR_CLOCKWISE
    elif (speed < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail at %d µm/s" % (speed))
        direction = L6470.DIR_COUNTER_CLOCKWISE
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Move speed = 0; ignoring command.")
    
    # Move the stepper.
    # TODO: convert speed to step/s
    _controller.run(direction, stepSpeed)

# Move By command
# Move the rail in the given direction by the given amount of distance.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# distance is reached.
# 
# speed: the distance to move the rail, in µm
def moveByCommand(distance):
    if (distance > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail by %d µm" % (distance))
        direction = L6470.DIR_CLOCKWISE
    elif (distance < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail by %d µm" % (distance))
        direction = L6470.DIR_COUNTER_CLOCKWISE
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Distance = 0; ignoring command.")
        
    # Move the stepper by the given distance.
    # TODO: convert distance to steps
    _controller.move(direction, steps)

# Move To command
# Move the rail to the given position.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# position is reached.
# 
# position: the position to move the rail to, in µm
def moveToCommand(position):
    # TODO: compare with current position to determine direction.
    
    if (delta > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail to %d µm" % (position))
        direction = L6470.DIR_CLOCKWISE
    elif (delta < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail to %d µm" % (position))
        direction = L6470.DIR_COUNTER_CLOCKWISE
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Distance = 0; ignoring command.")

    # Move the stepper to the given position.
    # TODO: convert position to step position
    _controller.goToDir(direction, stepPos)

# Stop the rail, either immediately or after a deceleration curve.
# If the rail is not currently moving, this command does nothing.
# 
# type: The type of stop (either "hard" or "soft")
def stopCommand(type):
    # Issue a stop command to the stepper controller.
    if (type == "hard"):
        rospy.loginfo(rospy.get_caller_id() + ": Hard stop")
        _controller.hardStop()
    elif (type == "soft"):
        rospy.loginfo(rospy.get_caller_id() + ": Soft stop")
        _controller.softStop()
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized stop type for \"%s\"" % (type))

# Main node function.
if __name__ == '__main__':
    # Open the SPI stepper controller.
    _controller = L6470()
    _controller.open(0, 0)

    # Initialize controller.
    _controller.status() # Must be done if the controller was in overcurrent alarm.
    _controller.hardDisengage() # If the controller is powered, settings are ignored.
    _controller.setStepMode(L6470.STEP_SEL_128)
    # TODO: Set to hardstop on SW detect.
    _controller.setThresholdSpeed(15600)
    _controller.setOCDThreshold(0x04)
    _controller.setStartSlope(0x0000)
    _controller.setIntersectSpeed(0x0000)
    _controller.setAccFinalSlope(63)
    _controller.setKvalHold(25)
    _controller.setKvalRun(55)
    _controller.setKvalAcc(55)
    _controller.setKvalDec(55)
    _controller.setMaxSpeed(700)

    rospy.init_node("rail_controller_node", anonymous=True)
    rospy.Subscriber("motor/inter_eye/command", String, messageCallback)

    rospy.spin()

    _controller.close()
