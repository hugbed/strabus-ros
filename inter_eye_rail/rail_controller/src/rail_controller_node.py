#! /usr/bin/env python

# ROS packages.
import rospy

# Standard messages.
from std_msgs.msg import String

# Controller library.
from L6470_pkg.L6470_lib import L6470

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

# Callback used when the controller rises a flag event.
def flagCallback(channel):
    # Determine the alarm that cause the interrupt.
    status = _controller.status()
    if (status & L6470.STATUS_SW_EN):
        # Reverse previous direction.
        if (_lastDirection == L6470.DIR_CLOCKWISE):
            _lastDirection = L6470.DIR_COUNTER_CLOCKWISE
            
            # Opening limit switch was hit: set position as mark.
            action = L6470.ACT_COPY
        elif (_lastDirection == L6470.DIR_COUNTER_CLOCKWISE):
            _lastDirection = L6470.DIR_CLOCKWISE
            
            # Closing limit switch was hit: set position as home.
            action = L6470.ACT_RESET
        else:
            print "ERROR: Controller node in invalid direction state."
            return
         
        _controller.releaseSW(_lastDirection)
  
# Interrupt script execution when a falling edge occurs on input 23. 
GPIO.add_event_detect(17, GPIO.FALLING, callback=flagCallback)

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
        if command == "move":
            runCommand(parameters['speed'])
        elif command == "moveBy":
            moveCommand(parameters['distance'])
        elif command == "moveTo":
            goToCommand(parameters['position'])
        elif command == "stop":
            stopCommand(parameters['type'])
        else:
            rospy.logerr(rospy.get_caller_id() + ": Unrecognized command \"%s\"" % (command))
    except ValueError as e:
        rospy.logerr(rospy.get_caller_id() + ": Error decoding JSON \"%s\"" % (str(e)))

# Calibrate command
# 
def calibrateCommand():
    _lastDirection = L6470.DIR_CLOCKWISE
    _controller.run(_lastDirection, 5000)
    
    # TODO: Wait for limit switch

    _lastDirection = L6470.DIR_COUNTER_CLOCKWISE
    _controller.run(_lastDirection, 5000)
    
    # TODO: Wait for limit switch
    
    # TODO: move rail to default inter-pupillary position

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
    stepSpeed = abs(speed) * STEPS_PER_MICROMETER
    _lastDirection = direction
    _controller.run(direction, stepSpeed)

# Move By command
# Move the rail in the given direction by the given amount of distance.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# distance is reached.
# 
# distance: the distance to move the rail, in µm
def moveByCommand(distance):
    # TODO: Limit distance to calibrated rail inter-pupillary limits.
    if (distance > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail by %d µm" % (distance))
        direction = L6470.DIR_CLOCKWISE
    elif (distance < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail by %d µm" % (distance))
        direction = L6470.DIR_COUNTER_CLOCKWISE
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Distance = 0; ignoring command.")
        
    # Move the stepper by the given distance.
    steps = round(abs(distance) * MICROSTEPS_PER_MICROMETER)
    _lastDirection = direction
    _controller.move(direction, steps)

# Move To command
# Move the rail to the given position.
# The rail will move until a stop command is issued, a limit switch is hit or until the target 
# position is reached.
# 
# targetPos: the position to move the rail to, in µm
def moveToCommand(targetPos):
    # TODO: Limit position to calibrated rail inter-pupillary limits.
    curStep = _controller.currentPosition()
    curPos = (curStep * DISTANCE_PER_MICROSTEP) + DISTANCE_OFFSET
    targetStep = round((targetPos - curPos - DISTANCE_OFFSET) * MICROSTEPS_PER_MICROMETER)
    delta = targetStep - curStep
    
    if (delta > 0):
        rospy.loginfo(rospy.get_caller_id() + ": Opening the rail to %d µm" % (position))
        direction = L6470.DIR_CLOCKWISE
    elif (delta < 0):
        rospy.loginfo(rospy.get_caller_id() + ": Closing the rail to %d µm" % (position))
        direction = L6470.DIR_COUNTER_CLOCKWISE
    else:
        rospy.loginfo(rospy.get_caller_id() + ": Distance = 0; ignoring command.")

    # Move the stepper to the given position.
    _lastDirection = direction
    _controller.goToDir(direction, targetStep)

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

# =================================================================================================
# Main node function
# =================================================================================================
if __name__ == '__main__':
    # Open the SPI stepper controller.
    _controller = L6470()
    _controller.open(0, 0)

    # Initialize controller.
    _controller.status() # Must be done if the controller was in overcurrent alarm.
    _controller.hardDisengage() # If the controller is powered, settings are ignored.
    _controller.setStepMode(L6470.STEP_SEL_128)
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
    
    # Set the controller to hard stop on SW (limit switch) detect.
    config = _controller.getConfig()
    _controller.setConfig(config | L6470.CONFIG_SW_MODE_HARDSTOP)

    # Initialize ROS node.
    rospy.init_node("rail_controller_node", anonymous=True)
    rospy.Subscriber("motor/inter_eye/command", String, messageCallback)

    rospy.spin()

    _controller.close()
