#! /usr/bin/env python

# ROS packages.
import rospy

# Stepper generated messages.
import stepper_controller.msg

# Controller library.
import L6470 from L6470

# Callback used to run the stepper in the given direction and at the given speed.
def runCallback(run):
    rospy.loginfo(rospy.get_caller_id() + ": Run at %d step/s in %s rotation" % (run.speed, run.direction)

    # TODO: implement run

# Callback used to move the stepper to the given position.
def goToCallback(goTo):
    rospy.loginfo(rospy.get_caller_id() + ": Go to position %d" % (goTo.position))

    # TODO: implement moveTo

# Main node function.
if __name__ == '__main__':
    # Open SPI controller.
    _controller = L6470()
    _controller.open(0, 0)

    rospy.init_node("stepper_controller_node", anonymous=True)

    rospy.Subscriber("stepper_controller_run", Run, runCallback)
    rospy.Subscriber("stepper_controller_goTo", GoTo, goToCallback)

    rospy.spin()
    
    _controller.close()
