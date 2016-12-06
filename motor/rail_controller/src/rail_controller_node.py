#! /usr/bin/env python

# ROS packages.
import rospy

# Standard messages.
from std_msgs.msg import String

# Controller library.
from L6470_pkg.L6470_lib import L6470

# Sleeping tool
from time import sleep

# JSON tool
import json

# GPIO
import RPi.GPIO as GPIO

# =================================================================================================
# GPIO setup
# =================================================================================================
GPIO.setmode(GPIO.BCM)

# GPIO 23 is used for stepper controller interrupts, for example when a limit switch is hit. 
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO 24 is used for the RESET pin of the stepper controller. Must be on at all times, unless 
# a reset wants to be performed.
GPIO.setup(24, GPIO.OUT)

# Callback used when the controller rises a flag event.
def flagCallback(channel):
    # Determine the alarm that caused the interrupt, and reset status.
    status = _controller.getStatus()
    print "Interrupt: %s" % (bin(status))
    if (status & L6470.STATUS_SW_EVN):
        print "Switch turn on event"
        _flags._releasing = True
        direction = (status & L6470.STATUS_DIR) >> L6470.STATUS_DIR_SHIFT

        if (direction == L6470.DIR_REVERSE):
            # Opening limit switch was hit: reverse direction and set position as home.
            _controller.releaseSW(L6470.ACT_RESET, L6470.DIR_FORWARD)
            print "Waiting for home release..."
            while ((_controller.getStatus() & L6470.STATUS_SW_F) != 0):
                # Wait a bit and look again if the switch is released.
                print "%s" % (bin(_controller.getStatus()))
                sleep(0.2)

            print "Release done!"

            _flags._homeSet = True
        elif (direction == L6470.DIR_FORWARD):
            # Closing limit switch was hit: reverse direction and set position as mark.
            _controller.releaseSW(L6470.ACT_COPY, L6470.DIR_REVERSE)
            print "Waiting for mark release..."
            while ((_controller.getStatus() & L6470.STATUS_SW_F) != 0):
                # Wait a bit and look again if the switch is released.
                print "%s" % (bin(_controller.getStatus()))
                sleep(0.2)

            print "Release done!"
            _flags._markSet = True

        _flags._releasing = False

    print "End status: %s" % (bin(_controller.status()))

# Interrupt script execution when a falling edge occurs on input 23. 
GPIO.add_event_detect(23, GPIO.FALLING, callback=flagCallback)

# =================================================================================================
# Conversion constants
# =================================================================================================
# The number of full steps for each rotation of the stepper.
STEPS_PER_TURN = 200

# The number of microsteps for each rotation of the stepper.
# This value is in accordance with the configured step mode in the stepper initialization.
MICROSTEPS_PER_TURN = STEPS_PER_TURN * 128

# The distance traveled by the rail for each full turn of the stepper.
# Multiplied by 2 because both arms of the rail move away or towards each other for each move.
DISTANCE_PER_TURN = 2540 * 2

# The inter-pupillary distance traveled by each microstep.
DISTANCE_PER_MICROSTEP = DISTANCE_PER_TURN / MICROSTEPS_PER_TURN

# The inter-pupillary distance traveled by each full step.
DISTANCE_PER_STEP = DISTANCE_PER_TURN / STEPS_PER_TURN

# The number of microsteps in each micrometer of inter-pupillary distance traveled.
MICROSTEPS_PER_MICROMETER = MICROSTEPS_PER_TURN / DISTANCE_PER_TURN

# The number of full steps in each micrometer of inter-pupillary distance traveled.
STEPS_PER_MICROMETER = STEPS_PER_TURN / DISTANCE_PER_TURN

# The inter-pupillary distance when the rail is at microstep position 0.
DISTANCE_OFFSET = 0

# =================================================================================================
# Commands
# =================================================================================================
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
        if command == "calibrate":
            calibrateCommand()
        elif command == "move":
            moveCommand(parameters['direction'])
        elif command == "moveBy":
            moveByCommand(parameters['distance'], parameters['direction'])
        elif command == "moveTo":
            moveToCommand(parameters['position'])
        elif command == "stop":
            stopCommand(parameters['type'])
        else:
            rospy.logerr(rospy.get_caller_id() + ": Unrecognized command \"%s\"" % (command))
    except ValueError as e:
        rospy.logerr(rospy.get_caller_id() + ": Error decoding JSON \"%s\"" % (str(e)))

# Calibrate command
# Perform a system calibration from one limit switch to the other.
# The system is frozen to commands during that time.
def calibrateCommand():
    rospy.loginfo(rospy.get_caller_id() + ": Calibrating system...")

    _flags._homeSet = False
    _flags._markSet = False
    _flags._calibrating = True

    # Start calibration to home position.
    print "Setting home..."
    _controller.run(L6470.DIR_REVERSE, 200)
    while (not _flags._homeSet):
        # Wait a bit and look again if home was set.
        sleep(0.1)

    print "Setting mark..."
    _controller.run(L6470.DIR_FORWARD, 200)
    while (not _flags._markSet):
        # Wait a bit and look again if the controller is busy.
        sleep(0.1)

    print "Setting default position..."
    _controller.move(L6470.DIR_REVERSE, 5 * MICROSTEPS_PER_TURN)

    _flags._calibrating = False
    rospy.loginfo(rospy.get_caller_id() + ": Calibration DONE!")

# Move command
# Move the rail at the maximum configured speed in the given direction.
# The rail will move until a stop command is issued or until a limit switch is hit.
#
# direction : the direction of the movement
def moveCommand(direction):
    # Ignore the command if releasing from switch or calibrating.
    if (_flags._releasing or _flags._calibrating):
        return

    # This command must run on a calibrated system.
    if (not _flags._homeSet or not _flags._markSet):
        rospy.logwarn(rospy.get_caller_id() + ": The system is not calibrated: ignoring command.")
        return

    # Move to the rail limit according to the direction.
    if (direction == "OPEN"):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail")
        _controller.goHome()
    elif (direction == "CLOSE"):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail")
        _controller.goMark()
    else:
        rospy.logerr(rospy.get_caller_id() + "Move command: Unrecognized direction %s" % (direction))

# Move By command
# Move the rail in the given direction by the given amount of distance.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# distance is reached.
#
# distance:   the distance to move the rail, in micrometers
# direction : the direction of the movement
def moveByCommand(distance, direction):
    # Ignore the command if releasing from switch or calibrating.
    if (_flags._releasing or _flags._calibrating):
        return

    # This command must run on a calibrated system.
    if (not _flags._homeSet or not _flags._markSet):
        rospy.logwarn(rospy.get_caller_id() + ": The system is not calibrated: ignoring command.")
        return

    if (distance <= 0):
        rospy.logwarn(rospy.get_caller_id() + "Move By command: Distance <= 0; ignoring command.")
        return

    moveBy = round(distance * MICROSTEPS_PER_MICROMETER)
    curStep = _controller.currentPosition()

    if (direction == "OPEN"):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail by %d micrometers" % (distance))

        if (curStep - moveBy < 0):
            _controller.goHome()
        else:
            _controller.move(L6470.DIR_REVERSE, moveBy)
    elif (direction == "CLOSE"):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail by %d micrometers" % (distance))

        if (curStep + moveBy > _controller.getMark()):
            _controller.goMark()
        else:
            _controller.move(L6470.DIR_FORWARD, moveBy)
    else:
        rospy.logerr(rospy.get_caller_id() + "Move By command: Unrecognized direction %s" % (direction))

# Move To command
# Move the rail to the given position.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# position is reached.
#
# targetPos: the position to move the rail to, in micrometers
def moveToCommand(targetPos):
    # Ignore the command if releasing from switch or calibrating.
    if (_flags._releasing or _flags._calibrating):
        return

    # This command must run on a calibrated system.
    if (not _flags._homeSet or not _flags._markSet):
        rospy.logwarn(rospy.get_caller_id() + ": The system is not calibrated: ignoring command.")
        return

    curStep = _controller.currentPosition()
    curPos = (curStep * DISTANCE_PER_MICROSTEP) + DISTANCE_OFFSET
    targetStep = round((targetPos - curPos - DISTANCE_OFFSET) * MICROSTEPS_PER_MICROMETER)
    delta = targetStep - curStep

    if (delta < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail to %d micrometers" % (position))

        if (targetStep < 0):
            _controller.goHome()
        else:
            _controller.goToDir(L6470.DIR_REVERSE, targetStep)
    elif (delta > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail to %d micrometers" % (position))

        if (targetStep > _controller.getMark()):
            _controller.goMark()
        else:
            _controller.goToDir(L6470.DIR_FORWARD, targetStep)
    else:
        rospy.logwarn(rospy.get_caller_id() + "Move To command: Distance = 0; ignoring command.")

# Stop the rail, either immediately or after a deceleration curve.
# If the rail is not currently moving, this command does nothing.
#
# type: The type of stop (either "hard" or "soft")
def stopCommand(type):
    # Ignore the command if releasing from switch.
    if (_flags._releasing or _flags._calibrating):
        return

    # Issue a stop command to the stepper controller.
    if (type == "hard"):
        rospy.loginfo(rospy.get_caller_id() + ": Hard stop")
        _controller.hardStop()
    elif (type == "soft"):
        rospy.loginfo(rospy.get_caller_id() + ": Soft stop")
        _controller.softStop()
    else:
        rospy.logerr(rospy.get_caller_id() + ": Unrecognized stop type for \"%s\"" % (type))

# =================================================================================================
# Flags class used for inter-thread states verifications.
# =================================================================================================
class Flags(object):
    def __init__(self):
        self._homeSet = False
        self._markSet = False
        self._calibrating = False
        self._releasing = False

# =================================================================================================
# Main node function
# =================================================================================================
if __name__ == '__main__':
    print "Preparing to launch Inter-pupillary rail controller node..."
    
    # Open the SPI stepper controller.
    _controller = L6470()
    _controller.open(0, 0)
    
    # Enable reset pin.
    GPIO.output(24, 1)

    # Initialize controller.
    _controller.status() # Must be done if the controller was in overcurrent alarm.
    _controller.hardDisengage() # If the controller is powered, settings are ignored.
    _controller.setStepMode(L6470.STEP_SEL_128)
    _controller.setThresholdSpeed(15600)
    _controller.setOCDThreshold(0x05)
    _controller.setStartSlope(0x0000)
    _controller.setIntersectSpeed(0x0000)
    _controller.setAccFinalSlope(35)
    _controller.setDecFinalSlope(35)
    _controller.setKvalHold(5)
    _controller.setKvalRun(45)
    _controller.setKvalAcc(45)
    _controller.setKvalDec(45)
    _controller.setMinSpeed(199, False)
    _controller.setMaxSpeed(200)

    # State flags
    _flags = Flags()

    print "Status: %s" % (bin(_controller.status()))

    # Initialize ROS node.
    rospy.init_node("rail_controller_node", anonymous=True)
    rospy.Subscriber("motor/inter_eye/command", String, messageCallback)

    print "Rail controller node successfully launched!"

    rospy.spin()

    print "Status: %s" % (bin(_controller.status()))

    GPIO.output(24, 0)
    _controller.close()
