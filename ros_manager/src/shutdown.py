#!/usr/bin/env python
import rospy
import roslaunch
from subprocess import call
from std_msgs.msg import Bool 

def shutdownCallback(msg):
    shutdown_flag = msg.data
    print "message data: %s"%(shutdown_flag)
    if shutdown_flag:
        print "SYSTEM IS HALTING, TAKE COVER!"
        call("halt", shell=True)
    else:
        print "No shut down for now"

def listener():
    rospy.init_node('shutdown', anonymous=True)
    rospy.Subscriber('shutdown', Bool, shutdownCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
