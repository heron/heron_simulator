#!/usr/bin/env python

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from heron_msgs.msg import Drive
import rospy

def cmd_drive_callback(drive):
    global p_left
    global p_right

    send = FloatStamped()
    send.header.stamp = rospy.Time.now()

    rospy.loginfo("Publishing!")
    send.data = drive.left
    p_left.publish(send)

    send.data = drive.right
    p_right.publish(send)

def translate():
    global p_left
    global p_right

    p_left = rospy.Publisher("thrusters/1/input", FloatStamped, queue_size=1)
    p_right = rospy.Publisher("thrusters/0/input", FloatStamped, queue_size=1)

    cmd_drive_sub = rospy.Subscriber("cmd_drive", Drive, cmd_drive_callback)

    rospy.spin()

print "sfdsf"
if __name__ == '__main__':
    rospy.init_node("cmd_drive_to_thrusters")
    rospy.loginfo("Publishing!")
    translate()
