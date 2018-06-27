#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Nirzvi1 <guys@qualum.com>
# @copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
