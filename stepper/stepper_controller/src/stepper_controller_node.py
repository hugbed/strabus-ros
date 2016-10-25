#! /usr/bin/env python

# ROS packages.
import rospy

# Stepper generated messages.
from stepper_controller.msg import Run
from stepper_controller.msg import Move
from stepper_controller.msg import GoTo
from stepper_controller.msg import Stop

# Controller library.
from L6470_pkg.L6470_lib import L6470

# Callback used to run the stepper in the given direction and at the given speed.
# The stepper will run until a stop command is issued or until a limit switch is hit.
def runCallback(run):
    rospy.loginfo(rospy.get_caller_id() + ": Run at %d step/s in %s rotation" % (run.speed, run.direction))

    # Run the stepper if the direction is appropriate.
    if (run.direction == "clockwise"):
        _controller.run(L6470.DIR_CLOCKWISE, run.speed)
    elif (run.direction == "counter-clockwise"):
        _controller.run(L6470.DIR_COUNTER_CLOCKWISE, run.speed)
    else:
        return
    
# Callback used to move the stepper by the given number of microsteps.
def moveCallback(move):
    rospy.loginfo(rospy.get_caller_id() + ": Move by %d steps in %s rotation" % (move.steps, move.direction))

    # Issue a Move command to the stepper if the direction is appropriate.
    if (move.direction == "clockwise"):
        _controller.move(L6470.DIR_CLOCKWISE, move.steps)
    elif (move.direction == "counter-clockwise"):
        _controller.move(L6470.DIR_COUNTER_CLOCKWISE, move.steps)
    else:
        return

# Callback used to move the stepper to the given position.
def goToCallback(goTo):
    rospy.loginfo(rospy.get_caller_id() + ": Go to position %d" % (goTo.position))

    # Issue a GoTo command to the stepper.
    _controller.goTo(goTo.position)
    
# Callback used to stop the stepper, either immediately or after a deceleration curve.
def stopCallback(stop):
    rospy.loginfo(rospy.get_caller_id() + ": %s stop" % (stop.type))

    # Issue a stop command to the stepper.
    if (stop.type == "hard"):
        _controller.hardStop()
    elif (stop.type == "soft"):
        _controller.softStop()
    else:
        return

# Main node function.
if __name__ == '__main__':
    # Open SPI controller.
    _controller = L6470()
    _controller.open(0, 0)

    rospy.init_node("stepper_controller_node", anonymous=True)

    rospy.Subscriber("stepper_controller_run", Run, runCallback)
    rospy.Subscriber("stepper_controller_move", Move, moveCallback)
    rospy.Subscriber("stepper_controller_goTo", GoTo, goToCallback)
    rospy.Subscriber("stepper_controller_stop", Stop, stopCallback)

    rospy.spin()
    
    _controller.close()
