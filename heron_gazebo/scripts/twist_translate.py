#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3

def twist_cb(msg):
    global twist_msg
    global received

    twist_msg = msg

    if (twist_msg.linear.x < 0):
        twist_msg.linear.x = twist_msg.linear.x * 40
    else:
        twist_msg.linear.x = twist_msg.linear.x * 70

    twist_msg.angular.z *= 29

    received = True


def translate():
    global twist_msg
    global received

    rospy.init_node("twist_translator")

    received = False
    zero_message_sent = False

    twist_sub = rospy.Subscriber("interactive_wrench", Twist, twist_cb)
    wrench_pub = rospy.Publisher("cmd_wrench", Wrench, queue_size=1)

    # Ensure thrusters start at zero
    rospy.sleep(3)
    new_msg = Wrench()
    new_msg.force = Vector3(0, 0, 0)
    new_msg.torque = Vector3(0, 0, 0)
    wrench_pub.publish(new_msg)

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        new_msg = Wrench()
        new_msg.force = Vector3(0, 0, 0)
        new_msg.torque = Vector3(0, 0, 0)

        if received:
            new_msg.force = twist_msg.linear;
            new_msg.torque = twist_msg.angular;
            received = False
            zero_message_sent = False
            wrench_pub.publish(new_msg);
        elif not zero_message_sent:
            wrench_pub.publish(new_msg);
            zero_message_sent = True

        r.sleep()

if __name__ == '__main__':
    translate()
