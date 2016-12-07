#!/usr/bin/env python
import rospy
import roslaunch
from subprocess import call
from std_msgs.msg import Bool 

def shutdownCallback(data):
    print "SYSTEM IS HALTING, TAKE COVER!"
    cmd = "halt"
    rospy.loginfo(cmd)
    call(cmd)

def listener():
    rospy.init_node('shutdown', anonymous=True)
    rospy.Subscriber('shutdown', Bool, shutdownCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
