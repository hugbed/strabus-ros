#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

LCD_PIN = 24

def callback(data):
    show = data.data
    GPIO.output(LCD_PIN, show)
    
def listener():
    rospy.init_node('lcd_toggle', anonymous=True)
    rospy.Subscriber('show', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LCD_PIN, GPIO.OUT)
    GPIO.output(LCD_PIN, GPIO.HIGH)
    listener()

