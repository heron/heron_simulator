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

from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import rospy

soft_bias = np.identity(3)
hard_bias = np.zeros((3,))

def mag_cb(msg):
    # switch from NED to ENU
    temp = msg.magnetic_field.x
    msg.magnetic_field.x = msg.magnetic_field.y
    msg.magnetic_field.y = temp
    msg.magnetic_field.z = -msg.magnetic_field.z


    switched = Vector3Stamped()
    switched.header = msg.header
    switched.vector = msg.magnetic_field


    switch_mag.publish(switched)

    mag_val = np.zeros((3,))
    mag_val[0] = msg.magnetic_field.x - hard_bias[0]
    mag_val[1] = msg.magnetic_field.y - hard_bias[1]
    mag_val[2] = msg.magnetic_field.z - hard_bias[2]
    mag_val = np.matmul(soft_bias, mag_val)

    print mag_val

    msg.magnetic_field.x = mag_val[0]
    msg.magnetic_field.y = mag_val[1]
    msg.magnetic_field.z = mag_val[2]

    cal_mag.publish(msg)

def get_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return default

def interpret():
    global cal_mag
    global switch_mag
    global hard_bias
    global soft_bias

    rospy.init_node("mag_interpreter")

    cal_mag = rospy.Publisher("imu/mag", MagneticField, queue_size=1)
    switch_mag = rospy.Publisher("imu/mag_raw", Vector3Stamped, queue_size=1)

    rospy.loginfo("%s", str(get_param("~soft_iron_bias", "[1, 0, 0, 0, 1, 0, 0, 0, 1]")))
    soft_bias_str = get_param("~soft_iron_bias", "[1, 0, 0, 0, 1, 0, 0, 0, 1]")
    soft_bias = np.asarray([float(x) for x in soft_bias_str[1:-1].split(",")]).reshape((3,3))

    rospy.loginfo("%s", get_param("~soft_iron_bias", [1, 0, 0, 0, 1, 0, 0, 0, 1]))

    hard_bias[0] = get_param("/mag_bias/x", 0)
    hard_bias[1] = get_param("/mag_bias/y", 0)
    hard_bias[2] = get_param("/mag_bias/z", 0)

    mag_sub = rospy.Subscriber("imu/mag_sim", MagneticField, mag_cb)

    rospy.spin()

if __name__ == '__main__':
    interpret()
