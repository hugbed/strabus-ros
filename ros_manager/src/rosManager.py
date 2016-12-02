#!/usr/bin/env python
import rospy
import roslaunch
from subprocess import call
from std_msgs.msg import String

def rosLaunchCallback(data):
    with open("/home/pi/rosmanager.log", 'a') as f:
        cmd = "roslaunch %s &" % data.data
        rospy.loginfo(cmd)
        call(cmd, shell=True, stdout=f, stderr=f)

def listener():
    rospy.init_node('ros_manager', anonymous=True)
    rospy.Subscriber('roslaunch', String, rosLaunchCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
