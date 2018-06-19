#!/usr/bin/env python

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
