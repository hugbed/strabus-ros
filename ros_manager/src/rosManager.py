#!/usr/bin/env python
import rospy
import roslaunch
from subprocess import call
from std_msgs.msg import String

def rosLaunchCallback(data):
    cmd = "roslaunch %s" % data.data
    rospy.loginfo(cmd)
    call(cmd.split(' '))

def rosRunCallback(data):
    cmd = "rosrun %s" % data.data
    rospy.loginfo(cmd)
    call(cmd.split(' '))

def listener():
    rospy.init_node('ros_manager', anonymous=True)
    rospy.Subscriber('roslaunch', String, rosLaunchCallback)
    rospy.Subscriber('rosrun', String, rosRunCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
