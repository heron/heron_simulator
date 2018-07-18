#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Guy Stoppi <gstoppi@clearpathrobotics.com>
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

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from heron_msgs.msg import Drive
import rospy

def cmd_drive_callback(drive):
    global p_left
    global p_right

    send = FloatStamped()
    send.header.stamp = rospy.Time.now()

    send.data = drive.left
    p_left.publish(send)

    send.data = drive.right
    p_right.publish(send)

def translate():
    global p_left
    global p_right

    namespace = rospy.get_param("~namespace", "")

    if namespace is "":
        namespace = "heron"

    p_left = rospy.Publisher("/" + namespace + "/thrusters/1/input", FloatStamped, queue_size=1)
    p_right = rospy.Publisher("/" + namespace + "/thrusters/0/input", FloatStamped, queue_size=1)

    cmd_drive_sub = rospy.Subscriber("cmd_drive", Drive, cmd_drive_callback)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("cmd_drive_to_thrusters")
    translate()
